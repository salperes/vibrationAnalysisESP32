#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <Update.h>
#include <arduinoFFT.h>

#include "LIS2DW12_ESP32.h"
#include "api_handlers.h"
#include "config.h"
#include "html_pages.h"
#include <string.h>

static String versionJson()
{
  String s = "{";
  s += "\"version\":\"" APP_VERSION "\",";
  s += "\"hash\":\"" BUILD_HASH "\",";
  s += "\"built\":\"" __DATE__ " " __TIME__ "\"";
  s += "}";
  return s;
}
#define FFT_N 1024 // power of 2
#define FFT_WINDOW FFT_WIN_TYP_HANN

// ======================= I2C mutex =======================

// ======================= Timer =======================
static volatile uint32_t dueCount = 0;
static hw_timer_t *timer0 = nullptr;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&mux);
  dueCount++;
  portEXIT_CRITICAL_ISR(&mux);
}

bool startTimerHz(uint16_t hz)
{
  if (!hz)
    return false;
  uint32_t period_us = 1000000UL / hz;
  timer0 = timerBegin(0, 80, true); // 80MHz/80=1MHz tick
  timerAttachInterrupt(timer0, &onTimer, true);
  timerAlarmWrite(timer0, period_us, true);
  timerAlarmEnable(timer0);
  return true;
}

void stopTimer()
{
  if (!timer0)
    return;
  timerAlarmDisable(timer0);
  timerDetachInterrupt(timer0);
  timerEnd(timer0);
  timer0 = nullptr;
}

// State lives in app_state.cpp
static void resetLivePreviewState()
{
  g_liveLastMs = 0;
  for (int i = 0; i < 3; i++)
  {
    g_live_g[i] = 0;
    g_live_acc_mps2[i] = 0;
    g_live_vel_mmps[i] = 0;
    g_live_disp_mm[i] = 0;
  }
  g_live_mag_acc = 0;
  g_live_mag_vel_mmps = 0;
  g_live_mag_disp_mm = 0;
}

// ======================= Helpers =======================
static bool isSafeAccelFile(String p)
{
  if (!p.startsWith("/"))
    p = "/" + p;
  if (!p.startsWith("/accel"))
    return false;
  if (!p.endsWith(".dat"))
    return false;
  if (p.indexOf("..") >= 0)
    return false;
  if (p.indexOf("//") >= 0)
    return false;
  return true;
}

static bool fileExists(const String &path) { return LittleFS.exists(path); }

static bool isValidYYMMDDHHMMSS(const String &ts)
{
  if (ts.length() != 12)
    return false;
  for (size_t i = 0; i < 12; i++)
    if (ts[i] < '0' || ts[i] > '9')
      return false;
  return true;
}

static String makeNewFileNameFromUI(const String &ts12)
{
  String base = "/accel" + ts12;
  String path = base + ".dat";
  if (!LittleFS.exists(path))
    return path;

  for (int i = 1; i <= 99; i++)
  {
    char suf[8];
    snprintf(suf, sizeof(suf), "_%02d", i);
    String p2 = base + String(suf) + ".dat";
    if (!LittleFS.exists(p2))
      return p2;
  }
  return base + "_" + String(millis()) + ".dat";
}

static bool rewriteHeaderSamples(const String &path, uint32_t samplesWritten)
{
  if (!fileExists(path))
    return false;
  File f = LittleFS.open(path, "r+");
  if (!f)
    return false;

  FileHeaderV3 h{};
  size_t got = f.read((uint8_t *)&h, sizeof(h));
  if (got != sizeof(h))
  {
    f.close();
    return false;
  }
  h.samples = samplesWritten;
  f.seek(0, SeekSet);
  size_t wrote = f.write((uint8_t *)&h, sizeof(h));
  f.close();
  return wrote == sizeof(h);
}

static LIS2DW12::FullScale fsFromG(uint8_t fs_g)
{
  switch (fs_g)
  {
  case 2:
    return LIS2DW12::FullScale::G2;
  case 4:
    return LIS2DW12::FullScale::G4;
  case 8:
    return LIS2DW12::FullScale::G8;
  case 16:
    return LIS2DW12::FullScale::G16;
  default:
    return LIS2DW12::FullScale::G2;
  }
}
static uint8_t fsToByte(LIS2DW12::FullScale fs)
{
  switch (fs)
  {
  case LIS2DW12::FullScale::G2:
    return 2;
  case LIS2DW12::FullScale::G4:
    return 4;
  case LIS2DW12::FullScale::G8:
    return 8;
  case LIS2DW12::FullScale::G16:
    return 16;
  }
  return 2;
}

static const char *poseName(int step)
{
  switch (step)
  {
  case 0:
    return "X+";
  case 1:
    return "X-";
  case 2:
    return "Y+";
  case 3:
    return "Y-";
  case 4:
    return "Z+";
  case 5:
    return "Z-";
  default:
    return "-";
  }
}

// ---- Analysis helpers ----
static float mgPerLsb(uint8_t res_bits, uint8_t fs_g)
{
  // AN5038 Table 15
  const bool is12 = (res_bits == 12);
  switch (fs_g)
  {
  case 2:
    return is12 ? 0.976f : 0.244f;
  case 4:
    return is12 ? 1.952f : 0.488f;
  case 8:
    return is12 ? 3.904f : 0.976f;
  case 16:
    return is12 ? 7.808f : 1.952f;
  default:
    return is12 ? 0.976f : 0.244f;
  }
}

static float rawAlignedToG(int16_t rawAligned, uint8_t res_bits, uint8_t fs_g)
{
  // aligned raw already right-aligned (12/14)
  float mg = mgPerLsb(res_bits, fs_g);
  return (float)rawAligned * (mg / 1000.0f);
}

static float applyCal1(float g, float offset, float scale)
{
  return (g - offset) * scale;
}

static float rmsFromSumSq(double sumSq, uint32_t n)
{
  return (n > 0) ? sqrtf((float)(sumSq / (double)n)) : 0.0f;
}

// ======================= File list cache =======================
static String buildFilesJsonNow()
{
  String out = "[";
  bool first = true;

  File root = LittleFS.open("/");
  if (!root)
    return "[]";
  if (!root.isDirectory())
  {
    root.close();
    return "[]";
  }

  File f = root.openNextFile();
  while (f)
  {
    String name = f.name();
    if (!name.startsWith("/"))
      name = "/" + name;

    if (isSafeAccelFile(name))
    {
      if (!first)
        out += ",";
      first = false;
      out += "{";
      out += "\"name\":\"" + name + "\",";
      out += "\"size\":" + String((uint32_t)f.size());
      out += "}";
    }

    f.close();
    f = root.openNextFile();
    delay(0);
  }
  root.close();

  out += "]";
  return out;
}

static String listFilesJsonCached()
{
  uint32_t now = millis();
  if (now - g_listCacheMs > LIST_CACHE_TTL_MS)
  {
    g_listCache = buildFilesJsonNow();
    g_listCacheMs = now;
  }
  return g_listCache;
}

void rebuildListCache()
{
  g_listCache = buildFilesJsonNow();
  g_listCacheMs = millis();
}

// ======================= FS info =======================
static String fsInfoJson()
{
  size_t total = LittleFS.totalBytes();
  size_t used = LittleFS.usedBytes();
  size_t freeB = (total >= used) ? (total - used) : 0;

  String s = "{";
  s += "\"total\":" + String((uint32_t)total) + ",";
  s += "\"used\":" + String((uint32_t)used) + ",";
  s += "\"free\":" + String((uint32_t)freeB);
  s += "}";
  return s;
}

// ======================= Recording task =======================
static void recordTask(void * /*arg*/)
{
  g_recording = true;
  g_stopRequested = false;
  g_samplesWritten = 0;
  g_maxBacklog = 0;
  g_elapsedMs = 0;

  String ts = g_uiTimestamp;
  String path = makeNewFileNameFromUI(ts);
  g_currentFile = path;

  if (g_i2cMutex)
    xSemaphoreTake(g_i2cMutex, portMAX_DELAY);

  Wire.setClock(1000000);
  LIS2DW12 lis(Wire, 0x18);
  if (!lis.begin(-1, -1, 1000000))
  {
    if (g_i2cMutex)
      xSemaphoreGive(g_i2cMutex);
    g_recording = false;
    g_recTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  LIS2DW12::Config cfg;
  cfg.mode = g_cfg.mode;
  cfg.lpMode = (g_cfg.mode == LIS2DW12::Mode::LowPower)
                   ? LIS2DW12::LowPowerMode::LP1_12bit
                   : LIS2DW12::LowPowerMode::LP2_14bit;
  cfg.fs = fsFromG(g_cfg.fs_g);
  cfg.lowNoise = true;
  cfg.bdu = true;
  cfg.autoInc = true;

  if (!lis.applyConfig(cfg))
  {
    if (g_i2cMutex)
      xSemaphoreGive(g_i2cMutex);
    g_recording = false;
    g_recTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  if (g_cfg.mode == LIS2DW12::Mode::LowPower && g_cfg.hz == 2)
  {
    lis.setPowerMode(LIS2DW12::Odr::Hz12_5_or_1_6,
                     LIS2DW12::Mode::LowPower,
                     LIS2DW12::LowPowerMode::LP1_12bit);
  }
  else
  {
    lis.setRateHz(g_cfg.hz);
  }

  lis.setOutputQuantization(g_cfg.qBits);
  lis.loadCalibrationNVS("lis2dw12", "cal");
  auto cal = lis.getCalibration();

  File f = LittleFS.open(path, "w");
  if (!f)
  {
    if (g_i2cMutex)
      xSemaphoreGive(g_i2cMutex);
    g_recording = false;
    g_recTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  FileHeaderV3 h{};
  memcpy(h.magic, "LIS2DW12", 8);
  h.version = 3;
  h.rate_hz = g_cfg.hz;
  h.record_s = g_cfg.sec;
  h.samples = 0;
  h.fs_g = fsToByte(cfg.fs);
  h.res_bits = lis.activeResolutionBits();
  h.q_bits = g_cfg.qBits;
  for (int i = 0; i < 3; i++)
  {
    h.cal_offset_g[i] = cal.offset_g[i];
    h.cal_scale[i] = cal.scale[i];
  }

  if (f.write((uint8_t *)&h, sizeof(h)) != sizeof(h))
  {
    f.close();
    if (g_i2cMutex)
      xSemaphoreGive(g_i2cMutex);
    g_recording = false;
    g_recTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }
  f.close();

  const uint32_t targetN = (uint32_t)g_cfg.hz * (uint32_t)g_cfg.sec;
  const size_t CHUNK_N = 1024;
  Sample6 *chunk = (Sample6 *)malloc(CHUNK_N * sizeof(Sample6));
  if (!chunk)
  {
    if (g_i2cMutex)
      xSemaphoreGive(g_i2cMutex);
    g_recording = false;
    g_recTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  dueCount = 0;
  startTimerHz(g_cfg.hz);

  uint32_t tStart = millis();
  uint32_t idx = 0;
  uint32_t maxBacklog = 0;

  while (idx < targetN && !g_stopRequested)
  {
    uint32_t localDue = 0;
    portENTER_CRITICAL(&mux);
    localDue = dueCount;
    dueCount = 0;
    portEXIT_CRITICAL(&mux);

    if (localDue > maxBacklog)
      maxBacklog = localDue;

    while (localDue && idx < targetN && !g_stopRequested)
    {
      size_t fill = 0;

      while (localDue && fill < CHUNK_N && idx < targetN && !g_stopRequested)
      {
        int16_t ax, ay, az;
        if (lis.readRawAligned(ax, ay, az))
        {
          chunk[fill].ax = ax;
          chunk[fill].ay = ay;
          chunk[fill].az = az;
          fill++;
          idx++;
        }
        localDue--;
      }

      if (fill)
      {
        File wf = LittleFS.open(path, "a");
        if (!wf)
          break;
        size_t bytes = fill * sizeof(Sample6);
        size_t wrote = wf.write((uint8_t *)chunk, bytes);
        wf.close();
        if (wrote != bytes)
          break;
        g_samplesWritten = idx;
      }
    }

    g_elapsedMs = millis() - tStart;
    delay(0);
  }

  stopTimer();
  free(chunk);

  g_samplesWritten = idx;
  g_maxBacklog = maxBacklog;
  g_elapsedMs = millis() - tStart;

  rewriteHeaderSamples(path, idx);

  rebuildListCache();

  if (g_i2cMutex)
    xSemaphoreGive(g_i2cMutex);

  g_recording = false;
  g_recTask = nullptr;
  vTaskDelete(nullptr);
}

// ======================= Calibration tasks =======================
static void calibrateStaticTask(void * /*arg*/)
{
  g_calibratingStatic = true;

  if (g_i2cMutex)
    xSemaphoreTake(g_i2cMutex, portMAX_DELAY);

  Wire.setClock(400000);
  LIS2DW12 lis(Wire, 0x18);
  bool ok = lis.begin(-1, -1, 400000);
  if (ok)
  {
    LIS2DW12::Config cfg;
    cfg.mode = LIS2DW12::Mode::HighPerf;
    cfg.lpMode = LIS2DW12::LowPowerMode::LP2_14bit;
    cfg.fs = LIS2DW12::FullScale::G2;
    cfg.lowNoise = true;
    cfg.bdu = true;
    cfg.autoInc = true;
    lis.applyConfig(cfg);
    lis.setRateHz(100);

    ok = lis.calibrateStatic(600, 5, 1.0f);
    if (ok)
    {
      lis.saveCalibrationNVS("lis2dw12", "cal");
      g_calDirty = true;
    }
  }

  if (g_i2cMutex)
    xSemaphoreGive(g_i2cMutex);
  g_calibratingStatic = false;
  resetLivePreviewState();
  vTaskDelete(nullptr);
}

static void calibrate6PosTask(void * /*arg*/)
{
  g_calibrating6 = true;
  g_calibStep = 0;

  if (g_i2cMutex)
    xSemaphoreTake(g_i2cMutex, portMAX_DELAY);

  Wire.setClock(400000);
  LIS2DW12 lis(Wire, 0x18);
  if (!lis.begin(-1, -1, 400000))
  {
    if (g_i2cMutex)
      xSemaphoreGive(g_i2cMutex);
    g_calibrating6 = false;
    g_calibStep = -1;
    vTaskDelete(nullptr);
    return;
  }

  LIS2DW12::Config cfg;
  cfg.mode = LIS2DW12::Mode::HighPerf;
  cfg.lpMode = LIS2DW12::LowPowerMode::LP2_14bit;
  cfg.fs = LIS2DW12::FullScale::G2;
  cfg.lowNoise = true;
  cfg.bdu = true;
  cfg.autoInc = true;
  lis.applyConfig(cfg);
  lis.setRateHz(100);

  for (int pose = 0; pose < 6; pose++)
  {
    g_calibStep = pose;
    vTaskDelay(pdMS_TO_TICKS(1200));

    float avg[3] = {0, 0, 0};
    if (!lis.collectPoseAverage((LIS2DW12::Pose)pose, avg, 700, 5))
    {
      if (g_i2cMutex)
        xSemaphoreGive(g_i2cMutex);
      g_calibrating6 = false;
      g_calibStep = -1;
      vTaskDelete(nullptr);
      return;
    }
    g_calibAvg[pose][0] = avg[0];
    g_calibAvg[pose][1] = avg[1];
    g_calibAvg[pose][2] = avg[2];
  }

  lis.calibrate6PositionFromAverages(g_calibAvg);
  lis.saveCalibrationNVS("lis2dw12", "cal");
  g_calDirty = true;

  if (g_i2cMutex)
    xSemaphoreGive(g_i2cMutex);

  g_calibStep = -1;
  g_calibrating6 = false;
  resetLivePreviewState();
  vTaskDelete(nullptr);
}

static String infoJson()
{
  String s = "{";
  s += "\"recording\":";
  s += (g_recording ? "true" : "false");
  s += ",";
  s += "\"hz\":";
  s += g_cfg.hz;
  s += ",";
  s += "\"fs_g\":";
  s += g_cfg.fs_g;
  s += ",";
  s += "\"sec\":";
  s += g_cfg.sec;
  s += ",";
  s += "\"samples\":";
  s += (uint32_t)g_samplesWritten;
  s += ",";
  s += "\"maxBacklog\":";
  s += (uint32_t)g_maxBacklog;
  s += ",";
  s += "\"elapsedMs\":";
  s += (uint32_t)g_elapsedMs;
  s += ",";
  s += "\"currentFile\":\"" + g_currentFile + "\",";
  s += "\"mode\":\"";
  s += (g_cfg.mode == LIS2DW12::Mode::LowPower ? "LP" : g_cfg.mode == LIS2DW12::Mode::HighPerf ? "HP"
                                                                                               : "OD");
  s += "\",";

  s += "\"calibratingStatic\":";
  s += (g_calibratingStatic ? "true" : "false");
  s += ",";
  s += "\"calibrating6\":";
  s += (g_calibrating6 ? "true" : "false");
  s += ",";
  s += "\"calibStep\":";
  s += g_calibStep;
  s += ",";
  s += "\"calibPose\":\"";
  s += poseName(g_calibStep);
  s += "\",";

  s += "\"apMode\":";
  s += (g_apMode ? "true" : "false");
  s += ",";
  s += "\"apSsid\":\"" + g_apSsid + "\"";

  s += "}";
  return s;
}

static bool parseHzFromUI(uint16_t uiHz, uint16_t &outHz, LIS2DW12::Mode &mode)
{
  if (uiHz == 2)
  {
    outHz = 2;
    mode = LIS2DW12::Mode::LowPower;
    return true;
  }
  const uint16_t allowed[] = {13, 25, 50, 100, 200, 400, 800, 1600};
  for (auto v : allowed)
    if (uiHz == v)
    {
      outHz = v;
      mode = LIS2DW12::Mode::HighPerf;
      return true;
    }
  return false;
}

// ======================= Handlers =======================
void handleRoot() { server.send(200, "text/html", INDEX_HTML); }
void handlePing() { server.send(200, "text/plain", "PONG"); }
void handleApiInfo() { server.send(200, "application/json", infoJson()); }
void handleApiList() { server.send(200, "application/json", listFilesJsonCached()); }
void handleApiFsInfo() { server.send(200, "application/json", fsInfoJson()); }

void handleApiStart()
{
  if (g_recording || g_calibratingStatic || g_calibrating6)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }

  uint16_t uiHz = server.hasArg("hz") ? (uint16_t)server.arg("hz").toInt() : 100;
  uint16_t sec = server.hasArg("sec") ? (uint16_t)server.arg("sec").toInt() : 60;
  uint8_t fs_g = server.hasArg("fs") ? (uint8_t)server.arg("fs").toInt() : 2;

  String ts = server.hasArg("ts") ? server.arg("ts") : "";
  if (!isValidYYMMDDHHMMSS(ts))
  {
    server.send(400, "text/plain", "Invalid ts (need YYMMDDHHMMSS from browser)");
    return;
  }

  const uint16_t allowedSec[] = {15, 30, 45, 60, 75, 90, 120, 180};
  bool secOk = false;
  for (auto v : allowedSec)
    if (sec == v)
      secOk = true;
  if (!secOk)
  {
    server.send(400, "text/plain", "Invalid sec");
    return;
  }

  if (!(fs_g == 2 || fs_g == 4 || fs_g == 8 || fs_g == 16))
  {
    server.send(400, "text/plain", "Invalid fs");
    return;
  }

  uint16_t hz = 100;
  LIS2DW12::Mode mode = LIS2DW12::Mode::HighPerf;
  if (!parseHzFromUI(uiHz, hz, mode))
  {
    server.send(400, "text/plain", "Invalid hz");
    return;
  }

  g_cfg.hz = hz;
  g_cfg.sec = sec;
  g_cfg.fs_g = fs_g;
  g_cfg.qBits = 0;
  g_cfg.mode = mode;
  g_uiTimestamp = ts;

  g_stopRequested = false;
  BaseType_t ok = xTaskCreatePinnedToCore(recordTask, "rec", 8192, nullptr, 2, &g_recTask, 1);
  if (ok != pdPASS)
  {
    server.send(500, "text/plain", "Task create failed");
    return;
  }

  server.send(200, "text/plain", "OK started");
}

void handleApiStop()
{
  if (!g_recording)
  {
    server.send(200, "text/plain", "Not recording");
    return;
  }
  g_stopRequested = true;
  server.send(200, "text/plain", "OK stop requested");
}

void handleDownload()
{
  if (!server.hasArg("file"))
  {
    server.send(400, "text/plain", "Missing file");
    return;
  }
  String path = server.arg("file");
  if (!path.startsWith("/"))
    path = "/" + path;
  if (!isSafeAccelFile(path))
  {
    server.send(400, "text/plain", "Bad file");
    return;
  }
  if (!fileExists(path))
  {
    server.send(404, "text/plain", "Not found");
    return;
  }

  File f = LittleFS.open(path, "r");
  if (!f)
  {
    server.send(500, "text/plain", "Open failed");
    return;
  }

  String basename = path;
  if (basename.startsWith("/"))
    basename.remove(0, 1);
  server.sendHeader("Content-Type", "application/octet-stream");
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + basename + "\"");
  server.sendHeader("Connection", "close");
  server.streamFile(f, "application/octet-stream");
  f.close();
}

void handleApiAnalyze()
{
  if (!server.hasArg("file"))
  {
    server.send(400, "text/plain", "Missing file");
    return;
  }

  String path = server.arg("file");
  if (!path.startsWith("/"))
    path = "/" + path;
  if (!isSafeAccelFile(path))
  {
    server.send(400, "text/plain", "Bad file");
    return;
  }
  if (!fileExists(path))
  {
    server.send(404, "text/plain", "Not found");
    return;
  }

  // Busy durumunda bile analiz yapılabilir, ama istersen kısıtlayabilirsin:
  if (g_recording)
  {
    server.send(409, "text/plain", "Recording in progress");
    return;
  }

  File f = LittleFS.open(path, "r");
  if (!f)
  {
    server.send(500, "text/plain", "Open failed");
    return;
  }
  if (f.size() < (int)sizeof(FileHeaderV3))
  {
    f.close();
    server.send(400, "text/plain", "Bad file");
    return;
  }

  FileHeaderV3 h{};
  if (f.read((uint8_t *)&h, sizeof(h)) != sizeof(h))
  {
    f.close();
    server.send(400, "text/plain", "Read header failed");
    return;
  }

  if (memcmp(h.magic, "LIS2DW12", 8) != 0)
  {
    f.close();
    server.send(400, "text/plain", "Bad magic");
    return;
  }

  // gerçek sample sayısını dosya boyutuna göre limitliyoruz
  const uint32_t headerBytes = sizeof(FileHeaderV3);
  const uint32_t sampleBytes = sizeof(Sample6);
  const uint32_t maxPossibleSamples = (uint32_t)((f.size() - headerBytes) / sampleBytes);
  uint32_t n = h.samples;
  if (n == 0 || n > maxPossibleSamples)
    n = maxPossibleSamples;

  // Downsample hedefi
  const uint32_t MAXPTS = 2000;
  uint32_t pts = (n <= MAXPTS) ? n : MAXPTS;
  if (pts < 2)
    pts = n; // çok küçükse

  // bucket step
  const double step = (pts > 0) ? ((double)n / (double)pts) : 1.0;

  // Bucket akümülatörleri (float)
  float *sumX = (float *)calloc(pts, sizeof(float));
  float *sumY = (float *)calloc(pts, sizeof(float));
  float *sumZ = (float *)calloc(pts, sizeof(float));
  uint16_t *cnt = (uint16_t *)calloc(pts, sizeof(uint16_t));

  if (!sumX || !sumY || !sumZ || !cnt)
  {
    if (sumX)
      free(sumX);
    if (sumY)
      free(sumY);
    if (sumZ)
      free(sumZ);
    if (cnt)
      free(cnt);
    f.close();
    server.send(500, "text/plain", "OOM");
    return;
  }

  // Stats
  float minX = +INFINITY, minY = +INFINITY, minZ = +INFINITY;
  float maxX = -INFINITY, maxY = -INFINITY, maxZ = -INFINITY;
  double ssX = 0, ssY = 0, ssZ = 0;

  // iterate
  Sample6 s{};
  uint32_t i = 0;
  while (i < n && f.read((uint8_t *)&s, sizeof(s)) == sizeof(s))
  {
    float gx = rawAlignedToG(s.ax, h.res_bits, h.fs_g);
    float gy = rawAlignedToG(s.ay, h.res_bits, h.fs_g);
    float gz = rawAlignedToG(s.az, h.res_bits, h.fs_g);

    gx = applyCal1(gx, h.cal_offset_g[0], h.cal_scale[0]);
    gy = applyCal1(gy, h.cal_offset_g[1], h.cal_scale[1]);
    gz = applyCal1(gz, h.cal_offset_g[2], h.cal_scale[2]);

    if (gx < minX)
      minX = gx;
    if (gx > maxX)
      maxX = gx;
    ssX += (double)gx * (double)gx;
    if (gy < minY)
      minY = gy;
    if (gy > maxY)
      maxY = gy;
    ssY += (double)gy * (double)gy;
    if (gz < minZ)
      minZ = gz;
    if (gz > maxZ)
      maxZ = gz;
    ssZ += (double)gz * (double)gz;

    uint32_t b = (pts <= 1) ? 0 : (uint32_t)floor((double)i / step);
    if (b >= pts)
      b = pts - 1;
    sumX[b] += gx;
    sumY[b] += gy;
    sumZ[b] += gz;
    if (cnt[b] < 65535)
      cnt[b]++;

    i++;
    if ((i & 0x3FF) == 0)
      delay(0); // watchdog friendly
  }
  f.close();

  const uint32_t usedN = i;
  const float rmsX = rmsFromSumSq(ssX, usedN);
  const float rmsY = rmsFromSumSq(ssY, usedN);
  const float rmsZ = rmsFromSumSq(ssZ, usedN);

  // JSON üret (stream)
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Type", "application/json");
  server.sendHeader("Connection", "close");
  server.send(200);

  // meta + stats
  String head;
  head.reserve(1024);
  head += "{";
  head += "\"file\":\"" + path + "\",";
  head += "\"rate_hz\":" + String(h.rate_hz) + ",";
  head += "\"record_s\":" + String(h.record_s) + ",";
  head += "\"samples_header\":" + String(h.samples) + ",";
  head += "\"samples_used\":" + String(usedN) + ",";
  head += "\"fs_g\":" + String(h.fs_g) + ",";
  head += "\"res_bits\":" + String(h.res_bits) + ",";
  head += "\"q_bits\":" + String(h.q_bits) + ",";
  head += "\"min\":[" + String(minX, 6) + "," + String(minY, 6) + "," + String(minZ, 6) + "],";
  head += "\"max\":[" + String(maxX, 6) + "," + String(maxY, 6) + "," + String(maxZ, 6) + "],";
  head += "\"rms\":[" + String(rmsX, 6) + "," + String(rmsY, 6) + "," + String(rmsZ, 6) + "],";
  head += "\"pts\":" + String(pts) + ",";
  // downsample sonrası efektif örnekleme (yaklaşık)
  float effHz = (pts > 1 && usedN > 1) ? (float)h.rate_hz * ((float)pts / (float)usedN) : (float)h.rate_hz;
  head += "\"eff_hz\":" + String(effHz, 4) + ",";
  head += "\"ax\":[";
  server.sendContent(head);

  // arrays
  auto sendFloatArray = [&](float *sum, uint16_t *c, bool closeArr, const char *nextKey)
  {
    String chunk;
    chunk.reserve(2048);
    for (uint32_t k = 0; k < pts; k++)
    {
      float v = (c[k] ? (sum[k] / (float)c[k]) : 0.0f);
      chunk += String(v, 6);
      if (k + 1 < pts)
        chunk += ",";
      if (chunk.length() > 1800)
      {
        server.sendContent(chunk);
        chunk = "";
        delay(0);
      }
    }
    if (chunk.length())
      server.sendContent(chunk);

    if (closeArr)
    {
      server.sendContent("]");
      if (nextKey)
      {
        server.sendContent(String(",\"") + nextKey + "\":[");
      }
    }
  };

  sendFloatArray(sumX, cnt, true, "ay");
  sendFloatArray(sumY, cnt, true, "az");
  sendFloatArray(sumZ, cnt, true, nullptr);

  server.sendContent("}");
  server.sendContent("");

  free(sumX);
  free(sumY);
  free(sumZ);
  free(cnt);
}

// --- CSV download handler (senin V3’teki aynı; burada kısaltmadım) ---
void handleDownloadCSV(); // forward decl (aşağıda aynen devam edeceksin)

void handleApiDelete()
{
  if (g_recording || g_calibratingStatic || g_calibrating6)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }
  if (!server.hasArg("file"))
  {
    server.send(400, "text/plain", "Missing file");
    return;
  }

  String path = server.arg("file");
  if (!path.startsWith("/"))
    path = "/" + path;
  if (!isSafeAccelFile(path))
  {
    server.send(400, "text/plain", "Bad file");
    return;
  }
  if (!fileExists(path))
  {
    server.send(404, "text/plain", "Not found");
    return;
  }

  LittleFS.remove(path);
  rebuildListCache();

  server.send(200, "text/plain", "Deleted");
}

void handleApiCalibrateStatic()
{
  if (g_recording || g_calibratingStatic || g_calibrating6)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }
  BaseType_t ok = xTaskCreatePinnedToCore(calibrateStaticTask, "calS", 4096, nullptr, 2, nullptr, 1);
  if (ok != pdPASS)
  {
    server.send(500, "text/plain", "Task create failed");
    return;
  }
  server.send(200, "text/plain", "Static calibration started");
}

void handleApiCalibrate6()
{
  if (g_recording || g_calibratingStatic || g_calibrating6)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }
  BaseType_t ok = xTaskCreatePinnedToCore(calibrate6PosTask, "cal6", 6144, nullptr, 2, nullptr, 1);
  if (ok != pdPASS)
  {
    server.send(500, "text/plain", "Task create failed");
    return;
  }
  server.send(200, "text/plain", "6-pos calibration started");
}

void handleApiLive()
{
  if (g_recording || g_calibratingStatic || g_calibrating6)
  {
    server.send(200, "application/json", "{\"enabled\":false}");
    return;
  }

  uint32_t now = millis();
  if (now - g_liveLastMs < 1000)
  {
    String s = "{";
    s += "\"enabled\":true,";
    s += "\"hz\":" + String(LIVE_PREVIEW_HZ) + ",";
    s += "\"fc\":" + String(g_live_lp_cut_hz, 1) + ",";
    s += "\"ax\":" + String(g_live_acc_mps2[0], 3) + ",";
    s += "\"ay\":" + String(g_live_acc_mps2[1], 3) + ",";
    s += "\"az\":" + String(g_live_acc_mps2[2], 3) + ",";
    s += "\"mag\":" + String(g_live_mag_acc, 3) + ",";
    s += "\"vx_mmps\":" + String(g_live_vel_mmps[0], 2) + ",";
    s += "\"vy_mmps\":" + String(g_live_vel_mmps[1], 2) + ",";
    s += "\"vz_mmps\":" + String(g_live_vel_mmps[2], 2) + ",";
    s += "\"vmag_mmps\":" + String(g_live_mag_vel_mmps, 2) + ",";
    s += "\"dx_mm\":" + String(g_live_disp_mm[0], 2) + ",";
    s += "\"dy_mm\":" + String(g_live_disp_mm[1], 2) + ",";
    s += "\"dz_mm\":" + String(g_live_disp_mm[2], 2) + ",";
    s += "\"dmag_mm\":" + String(g_live_mag_disp_mm, 2);
    s += "}";
    server.send(200, "application/json", s);
    return;
  }
  g_liveLastMs = now;

  if (g_i2cMutex)
  {
    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE)
    {
      server.send(200, "application/json", "{\"enabled\":false}");
      return;
    }
  }

  Wire.setClock(1000000);
  LIS2DW12 lis(Wire, 0x18);
  if (!lis.begin(-1, -1, 1000000))
  {
    if (g_i2cMutex)
      xSemaphoreGive(g_i2cMutex);
    server.send(200, "application/json", "{\"enabled\":false}");
    return;
  }

  LIS2DW12::Config cfg;
  cfg.mode = LIS2DW12::Mode::HighPerf;
  cfg.lpMode = LIS2DW12::LowPowerMode::LP2_14bit;
  cfg.fs = LIS2DW12::FullScale::G2;
  cfg.lowNoise = true;
  cfg.bdu = true;
  cfg.autoInc = true;
  lis.applyConfig(cfg);
  lis.setRateHz(LIVE_PREVIEW_HZ);

  lis.loadCalibrationNVS("lis2dw12", "cal");
  g_calDirty = false;

  auto cal = lis.getCalibration();
  const uint8_t resBits = lis.activeResolutionBits();
  const uint8_t fs_g = fsToByte(cfg.fs);

  float cutoff = g_live_lp_cut_hz;
  if (server.hasArg("fc"))
  {
    float fcReq = server.arg("fc").toFloat();
    // clamp reasonable range
    if (fcReq >= 5.0f && fcReq <= (float)LIVE_PREVIEW_HZ * 0.5f)
      cutoff = fcReq;
  }
  // remember last used cutoff
  g_live_lp_cut_hz = cutoff;

  const uint16_t samples = LIVE_PREVIEW_HZ;
  const float dt = 1.0f / (float)LIVE_PREVIEW_HZ;
  const float tau = 1.0f / (2.0f * PI * cutoff);
  const float alpha = dt / (tau + dt); // 1st-order LPF coefficient

  double sumAcc[3] = {0, 0, 0};
  float vel[3] = {0, 0, 0};
  float disp[3] = {0, 0, 0};
  float lpf[3] = {0, 0, 0};
  bool lpfInit = false;
  uint16_t valid = 0;

  for (uint16_t i = 0; i < samples; i++)
  {
    int16_t axRaw, ayRaw, azRaw;
    if (!lis.readRawAligned(axRaw, ayRaw, azRaw))
    {
      delayMicroseconds(1200);
      continue;
    }

    float gx = rawAlignedToG(axRaw, resBits, fs_g);
    float gy = rawAlignedToG(ayRaw, resBits, fs_g);
    float gz = rawAlignedToG(azRaw, resBits, fs_g);

    gx = applyCal1(gx, cal.offset_g[0], cal.scale[0]);
    gy = applyCal1(gy, cal.offset_g[1], cal.scale[1]);
    gz = applyCal1(gz, cal.offset_g[2], cal.scale[2]);

    float ax = gx * GRAVITY_MPS2;
    float ay = gy * GRAVITY_MPS2;
    float az = gz * GRAVITY_MPS2;

    if (!lpfInit)
    {
      lpf[0] = ax;
      lpf[1] = ay;
      lpf[2] = az;
      lpfInit = true;
    }
    else
    {
      lpf[0] += alpha * (ax - lpf[0]);
      lpf[1] += alpha * (ay - lpf[1]);
      lpf[2] += alpha * (az - lpf[2]);
    }

    sumAcc[0] += lpf[0];
    sumAcc[1] += lpf[1];
    sumAcc[2] += lpf[2];

    vel[0] += lpf[0] * dt;
    vel[1] += lpf[1] * dt;
    vel[2] += lpf[2] * dt;

    disp[0] += vel[0] * dt;
    disp[1] += vel[1] * dt;
    disp[2] += vel[2] * dt;

    valid++;
    delayMicroseconds(1200); // attempt to stay close to sensor ODR
  }

  if (g_i2cMutex)
    xSemaphoreGive(g_i2cMutex);

  if (!valid)
  {
    g_liveLastMs = 0;
    server.send(200, "application/json", "{\"enabled\":false}");
    return;
  }

  const float invN = 1.0f / (float)valid;
  for (int i = 0; i < 3; i++)
  {
    g_live_acc_mps2[i] = (float)(sumAcc[i] * invN);
    g_live_vel_mmps[i] = vel[i] * 1000.0f;
    g_live_disp_mm[i] = disp[i] * 1000.0f;
    g_live_g[i] = g_live_acc_mps2[i] / GRAVITY_MPS2;
  }

  g_live_mag_acc = sqrtf(g_live_acc_mps2[0] * g_live_acc_mps2[0] +
                         g_live_acc_mps2[1] * g_live_acc_mps2[1] +
                         g_live_acc_mps2[2] * g_live_acc_mps2[2]);
  g_live_mag_vel_mmps = sqrtf(g_live_vel_mmps[0] * g_live_vel_mmps[0] +
                              g_live_vel_mmps[1] * g_live_vel_mmps[1] +
                              g_live_vel_mmps[2] * g_live_vel_mmps[2]);
  g_live_mag_disp_mm = sqrtf(g_live_disp_mm[0] * g_live_disp_mm[0] +
                             g_live_disp_mm[1] * g_live_disp_mm[1] +
                             g_live_disp_mm[2] * g_live_disp_mm[2]);

  String s = "{";
  s += "\"enabled\":true,";
  s += "\"hz\":" + String(LIVE_PREVIEW_HZ) + ",";
  s += "\"fc\":" + String(cutoff, 1) + ",";
  s += "\"ax\":" + String(g_live_acc_mps2[0], 3) + ",";
  s += "\"ay\":" + String(g_live_acc_mps2[1], 3) + ",";
  s += "\"az\":" + String(g_live_acc_mps2[2], 3) + ",";
  s += "\"mag\":" + String(g_live_mag_acc, 3) + ",";
  s += "\"vx_mmps\":" + String(g_live_vel_mmps[0], 2) + ",";
  s += "\"vy_mmps\":" + String(g_live_vel_mmps[1], 2) + ",";
  s += "\"vz_mmps\":" + String(g_live_vel_mmps[2], 2) + ",";
  s += "\"vmag_mmps\":" + String(g_live_mag_vel_mmps, 2) + ",";
  s += "\"dx_mm\":" + String(g_live_disp_mm[0], 2) + ",";
  s += "\"dy_mm\":" + String(g_live_disp_mm[1], 2) + ",";
  s += "\"dz_mm\":" + String(g_live_disp_mm[2], 2) + ",";
  s += "\"dmag_mm\":" + String(g_live_mag_disp_mm, 2);
  s += "}";
  server.send(200, "application/json", s);
}

void handleApiVersion() { server.send(200, "application/json", versionJson()); }

// ======================= NEW: RESET endpoint =======================
void handleApiReset()
{
  if (g_recording || g_calibratingStatic || g_calibrating6)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }
  server.send(200, "text/plain", "OK rebooting");
  delay(150);
  ESP.restart();
}

// ======================= NEW: Firmware update handlers =======================
static void handleUpdateGet()
{
  if (g_recording || g_calibratingStatic || g_calibrating6)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }
  server.send(200, "text/html", UPDATE_HTML);
}

void handleUpdatePost()
{
  if (g_updateLastError.length())
  {
    String msg = "FAIL: " + g_updateLastError;
    server.send(500, "text/plain", msg);
    return;
  }
  if (Update.hasError())
  {
    server.send(500, "text/plain", "FAIL: Update.hasError()");
    return;
  }

  server.send(200, "text/plain", "OK. Update success. Rebooting...");
  delay(250);
  ESP.restart();
}

void handleUpdateUpload()
{
  if (g_recording || g_calibratingStatic || g_calibrating6)
  {
    g_updateLastError = "Busy";
    Update.abort();
    return;
  }
  HTTPUpload &up = server.upload();

  if (up.status == UPLOAD_FILE_START)
  {
    g_updateLastError = "";
    g_updateExpected = up.totalSize;

    Serial.printf("[UPDATE] Start: %s, size=%u\n", up.filename.c_str(), (unsigned)up.totalSize);

    if (g_recording || g_calibratingStatic || g_calibrating6)
    {
      g_updateLastError = "Busy";
      return;
    }

    bool ok = Update.begin((up.totalSize > 0) ? up.totalSize : UPDATE_SIZE_UNKNOWN, U_FLASH);
    if (!ok)
    {
      g_updateLastError = "Update.begin() failed";
      Update.printError(Serial);
      Serial.println();
    }
  }
  else if (up.status == UPLOAD_FILE_WRITE)
  {
    if (g_updateLastError.length())
      return;

    size_t written = Update.write(up.buf, up.currentSize);
    if (written != up.currentSize)
    {
      g_updateLastError = "Update.write() failed";
      Update.printError(Serial);
      Serial.println();
    }
  }
  else if (up.status == UPLOAD_FILE_END)
  {
    if (g_updateLastError.length())
      return;

    bool ok = Update.end(true); // true = set boot partition
    if (!ok)
    {
      g_updateLastError = "Update.end() failed";
      Update.printError(Serial);
      Serial.println();
    }
    else
    {
      Serial.printf("[UPDATE] Success. Written=%u\n", (unsigned)up.totalSize);
    }
  }
  else if (up.status == UPLOAD_FILE_ABORTED)
  {
    g_updateLastError = "Upload aborted";
    Update.abort();
    Serial.println("[UPDATE] Aborted");
  }
}

void handleApiFFT()
{
  if (!server.hasArg("file") || !server.hasArg("axis"))
  {
    server.send(400, "text/plain", "Missing file or axis");
    return;
  }

  String path = server.arg("file");
  if (!path.startsWith("/"))
    path = "/" + path;
  if (!isSafeAccelFile(path) || !LittleFS.exists(path))
  {
    server.send(404, "text/plain", "Bad file");
    return;
  }

  char axis = server.arg("axis")[0]; // x y z
  int axisIdx = (axis == 'x') ? 0 : (axis == 'y') ? 1
                                : (axis == 'z')   ? 2
                                                  : -1;
  if (axisIdx < 0)
  {
    server.send(400, "text/plain", "Bad axis");
    return;
  }

  File f = LittleFS.open(path, "r");
  FileHeaderV3 h{};
  f.read((uint8_t *)&h, sizeof(h));

  uint32_t maxSamples = min((uint32_t)FFT_N, h.samples);
  if (maxSamples < 16)
  {
    f.close();
    server.send(400, "text/plain", "Too few samples");
    return;
  }

  static double vReal[FFT_N];
  static double vImag[FFT_N];

  memset(vImag, 0, sizeof(vImag));

  Sample6 s;
  for (uint32_t i = 0; i < maxSamples; i++)
  {
    if (f.read((uint8_t *)&s, sizeof(s)) != sizeof(s))
      break;

    int16_t raw =
        axisIdx == 0 ? s.ax : axisIdx == 1 ? s.ay
                                           : s.az;

    float g = rawAlignedToG(raw, h.res_bits, h.fs_g);
    g = applyCal1(g, h.cal_offset_g[axisIdx], h.cal_scale[axisIdx]);

    vReal[i] = g;
  }
  f.close();

  arduinoFFT FFT(vReal, vImag, maxSamples, h.rate_hz);
  FFT.Windowing(FFT_WINDOW, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  uint32_t bins = maxSamples / 2;
  double df = (double)h.rate_hz / (double)maxSamples;

  // Dominant frequency
  double peakMag = 0;
  double peakHz = 0;

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json");

  server.sendContent("{");
  server.sendContent("\"axis\":\"");
  server.sendContent(String(axis));
  server.sendContent("\",\"rate_hz\":");
  server.sendContent(String(h.rate_hz));
  server.sendContent(",\"df\":");
  server.sendContent(String(df, 6));
  server.sendContent(",\"fft\":[");

  for (uint32_t i = 1; i < bins; i++)
  {
    double mag = vReal[i];
    double hz = i * df;

    if (mag > peakMag)
    {
      peakMag = mag;
      peakHz = hz;
    }

    server.sendContent(String(mag, 6));
    if (i + 1 < bins)
      server.sendContent(",");
  }

  server.sendContent("],");
  server.sendContent("\"peak_hz\":");
  server.sendContent(String(peakHz, 3));
  server.sendContent(",");
  server.sendContent("\"peak_mag\":");
  server.sendContent(String(peakMag, 6));
  server.sendContent("}");
}

// ======================= Route registration =======================
void registerRoutes()
{
  server.on("/", handleRoot);
  server.on("/ping", handlePing);

  server.on("/api/info", handleApiInfo);
  server.on("/api/list", handleApiList);
  server.on("/api/fsinfo", handleApiFsInfo);

  server.on("/api/start", handleApiStart);
  server.on("/api/stop", handleApiStop);

  server.on("/download", handleDownload);
  server.on("/download_csv", handleDownloadCSV);
  server.on("/api/delete", handleApiDelete);

  server.on("/api/calibrate_static", handleApiCalibrateStatic);
  server.on("/api/calibrate6", handleApiCalibrate6);

  server.on("/api/live", handleApiLive);

  server.on("/api/reset", handleApiReset);
  server.on("/update", HTTP_GET, handleUpdateGet);
  server.on("/update", HTTP_POST, handleUpdatePost, handleUpdateUpload);

  server.on("/api/version", handleApiVersion);
  server.on("/api/analyze", handleApiAnalyze);
  server.on("/api/fft", handleApiFFT);
}

// ======================= CSV exporter (senin V3’tekiyle aynı) =======================
// Burayı senin V3’ten aynen bırakıyorum (tam fonksiyon gerekli).
void handleDownloadCSV()
{
  if (!server.hasArg("file"))
  {
    server.send(400, "text/plain", "Missing file");
    return;
  }
  String path = server.arg("file");
  if (!path.startsWith("/"))
    path = "/" + path;
  if (!isSafeAccelFile(path))
  {
    server.send(400, "text/plain", "Bad file");
    return;
  }
  if (!fileExists(path))
  {
    server.send(404, "text/plain", "Not found");
    return;
  }

  File f = LittleFS.open(path, "r");
  if (!f)
  {
    server.send(500, "text/plain", "Open failed");
    return;
  }
  if (f.size() < (int)sizeof(FileHeaderV3))
  {
    f.close();
    server.send(400, "text/plain", "Bad file");
    return;
  }

  FileHeaderV3 h{};
  if (f.read((uint8_t *)&h, sizeof(h)) != sizeof(h))
  {
    f.close();
    server.send(400, "text/plain", "Read header failed");
    return;
  }

  String basename = path;
  if (basename.startsWith("/"))
    basename.remove(0, 1);
  String csvName = basename;
  if (csvName.endsWith(".dat"))
    csvName = csvName.substring(0, csvName.length() - 4) + ".csv";

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + csvName + "\"");
  server.sendHeader("Connection", "close");
  server.send(200);

  String hdr;
  hdr.reserve(512);
  hdr += "# " + basename + "\n";
  hdr += "# rate_hz=" + String(h.rate_hz) + "\n";
  hdr += "# record_s=" + String(h.record_s) + "\n";
  hdr += "# samples=" + String(h.samples) + "\n";
  hdr += "# fs_g=" + String(h.fs_g) + "\n";
  hdr += "# res_bits=" + String(h.res_bits) + "\n";
  hdr += "# q_bits=" + String(h.q_bits) + "\n";
  hdr += "t_ms,ax_raw,ay_raw,az_raw\n";
  hdr += "# fs_g=" + String(h.fs_g) + "\n";
  hdr += "# res_bits=" + String(h.res_bits) + "\n";
  hdr += "# cal_offset_g=" + String(h.cal_offset_g[0], 6) + "," + String(h.cal_offset_g[1], 6) + "," + String(h.cal_offset_g[2], 6) + "\n";
  hdr += "# cal_scale=" + String(h.cal_scale[0], 6) + "," + String(h.cal_scale[1], 6) + "," + String(h.cal_scale[2], 6) + "\n";

  server.sendContent(hdr);

  const uint32_t dt_ms = (h.rate_hz > 0) ? (1000UL / h.rate_hz) : 0;
  uint32_t t_ms = 0;

  char line[96];
  Sample6 s;
  String chunk;
  chunk.reserve(2048);

  while (f.read((uint8_t *)&s, sizeof(s)) == sizeof(s))
  {
    int n = snprintf(line, sizeof(line), "%lu,%d,%d,%d\n",
                     (unsigned long)t_ms, (int)s.ax, (int)s.ay, (int)s.az);
    if (n > 0)
      chunk += String(line);

    t_ms += dt_ms;

    if (chunk.length() > 1800)
    {
      server.sendContent(chunk);
      chunk = "";
      delay(0);
    }
  }

  if (chunk.length())
    server.sendContent(chunk);
  server.sendContent("");
  f.close();
}



