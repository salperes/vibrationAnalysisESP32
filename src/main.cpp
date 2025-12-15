#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <arduinoFFT.h>

#include "LIS2DW12_ESP32.h"
// ======================= Build Info =======================
// İstersen platformio.ini veya Arduino build flags ile override edebilirsin:
// -D APP_VERSION="\"V3.1\"" -D BUILD_HASH="\"a1b2c3d\""
#ifndef APP_VERSION
#define APP_VERSION "V3.4"
#endif

#ifndef BUILD_HASH
#define BUILD_HASH "nogit"
#endif

#define LOG_SERIAL 0

static String versionJson()
{
  String s = "{";
  s += "\"version\":\"" APP_VERSION "\",";
  s += "\"hash\":\"" BUILD_HASH "\",";
  s += "\"built\":\"" __DATE__ " " __TIME__ "\"";
  s += "}";
  return s;
}

// ----------------- USER WIFI -----------------
const char *WIFI_SSID = "XXXX";
const char *WIFI_PASS = "XXXXX";
// ---------------------------------------------

#define FFT_N 1024 // power of 2
#define FFT_WINDOW FFT_WIN_TYP_HANN

static String g_updateLastError = "";
static size_t g_updateExpected = 0;

WebServer server(80);

// ======================= Data format (v3) =======================
#pragma pack(push, 1)
struct FileHeaderV3
{
  char magic[8];     // "LIS2DW12"
  uint16_t version;  // 3
  uint16_t rate_hz;  // selected
  uint16_t record_s; // selected
  uint32_t samples;  // actually written
  uint8_t fs_g;      // 2/4/8/16
  uint8_t res_bits;  // 12/14
  uint8_t q_bits;    // 0/10/12/14
  uint8_t reserved0;
  float cal_offset_g[3];
  float cal_scale[3];
};
struct Sample6
{
  int16_t ax, ay, az; // aligned raw
};
#pragma pack(pop)

// ======================= I2C mutex =======================
static SemaphoreHandle_t g_i2cMutex = nullptr;

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

// ======================= Logger state =======================
struct RecConfig
{
  uint16_t hz = 100; // UI: 2 -> 1.6Hz (LP special), 13 -> 12.5, etc.
  uint16_t sec = 60;
  uint8_t fs_g = 2;
  uint8_t qBits = 0;
  LIS2DW12::Mode mode = LIS2DW12::Mode::HighPerf; // LP/HP
};

static volatile bool g_recording = false;
static volatile bool g_stopRequested = false;

static volatile bool g_calibratingStatic = false;
static volatile bool g_calibrating6 = false;
static volatile int g_calibStep = -1; // 0..5

static RecConfig g_cfg;
static volatile uint32_t g_samplesWritten = 0;
static volatile uint32_t g_maxBacklog = 0;
static volatile uint32_t g_elapsedMs = 0;

static TaskHandle_t g_recTask = nullptr;
static String g_currentFile = ""; // "/accelYYMMDDHHMMSS.dat"
static String g_uiTimestamp = ""; // "YYMMDDHHMMSS"

// calibration wizard measurements
static float g_calibAvg[6][3] = {0};

// ======================= Live preview =======================
static uint32_t g_liveLastMs = 0;
static float g_live_g[3] = {0, 0, 0};
static volatile bool g_calDirty = true; // calibration changed -> reload NVS

// ======================= WiFi/AP state =======================
static bool g_apMode = false;
static String g_apSsid = "";

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
static String g_listCache = "[]";
static uint32_t g_listCacheMs = 0;
static const uint32_t LIST_CACHE_TTL_MS = 2000;

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

  g_listCache = buildFilesJsonNow();
  g_listCacheMs = millis();

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
  vTaskDelete(nullptr);
}

// ======================= Firmware Update Page =======================
static const char UPDATE_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Firmware Update</title>
  <style>
    body{font-family:system-ui,Segoe UI,Roboto,Arial;max-width:820px;margin:18px auto;padding:0 12px}
    .card{border:1px solid #ddd;border-radius:12px;padding:14px}
    h1{font-size:18px;margin:0 0 10px}
    .small{font-size:12px;color:#666}
    input,button{font-size:14px;padding:10px;border-radius:10px;border:1px solid #bbb}
    button{cursor:pointer}
    .row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
    .mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:12px}
    .bar{width:100%; height:18px}
    .status{margin-top:10px}
    .ok{color:#0a7} .warn{color:#c70} .bad{color:#c00}
  </style>
</head>
<body>
  <div class="card">
    <h1>ESP32 Firmware Update</h1>

    <div class="small">
      Current build:
      <span id="ver" class="mono">loading...</span>
    </div>

    <div class="small" style="margin-top:8px">
      Select the <b>.bin</b> built for this board/partition, then upload.
      Upload completes → device reboots.
    </div>

    <div class="row" style="margin-top:12px">
      <input id="file" type="file" accept=".bin" required/>
      <button id="btnUp" onclick="startUpload()">UPLOAD</button>
      <button onclick="location.href='/'">BACK</button>
    </div>

    <div style="margin-top:12px">
      <progress id="prog" class="bar" value="0" max="100"></progress>
      <div id="ptext" class="small mono">0%</div>
    </div>

    <div id="msg" class="status small">Ready.</div>

    <div class="small" style="margin-top:10px">
      During upload do not power off the device.
    </div>
  </div>

<script>
async function loadVersion(){
  try{
    const r = await fetch('/api/version', {cache:'no-store'});
    const j = await r.json();
    const v = `${j.version}  (${j.hash})  built: ${j.built}`;
    document.getElementById('ver').textContent = v;
  }catch(e){
    document.getElementById('ver').textContent = 'unknown';
  }
}

function setMsg(text, cls){
  const el = document.getElementById('msg');
  el.className = 'status small ' + (cls||'');
  el.textContent = text;
}

function setProgress(p){
  const prog = document.getElementById('prog');
  const ptext = document.getElementById('ptext');
  prog.value = p;
  ptext.textContent = `${p.toFixed(0)}%`;
}

function startUpload(){
  const f = document.getElementById('file').files[0];
  if(!f){ alert('Select a .bin file'); return; }

  // UI lock
  document.getElementById('btnUp').disabled = true;
  document.getElementById('file').disabled = true;
  setProgress(0);
  setMsg('Uploading...', 'warn');

  const form = new FormData();
  form.append('update', f, f.name);

  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/update', true);

  xhr.upload.onprogress = (e)=>{
    if(!e.lengthComputable) return;
    const p = (e.loaded / e.total) * 100.0;
    setProgress(p);
  };

  xhr.onload = ()=>{
    // ESP tarafı 200 text/plain dönüyor
    const txt = xhr.responseText || '';
    if(xhr.status === 200){
      setProgress(100);
      setMsg(txt + ' (page will disconnect)', 'ok');
      // reboot sonrası bağlantı kopacak; kullanıcı manuel yeniler
    } else {
      setMsg(`Upload failed: HTTP ${xhr.status} ${txt}`, 'bad');
      document.getElementById('btnUp').disabled = false;
      document.getElementById('file').disabled = false;
    }
  };

  xhr.onerror = ()=>{
    setMsg('Upload error (network).', 'bad');
    document.getElementById('btnUp').disabled = false;
    document.getElementById('file').disabled = false;
  };

  xhr.send(form);
}

loadVersion();
</script>
</body>
</html>
)HTML";

// ======================= UI HTML (V3) =======================
// (Senin V3 HTML'in aynen, sadece "RESET" ve "FIRMWARE UPDATE" butonları eklendi.)
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>ESP32 LIS2DW12 Recorder</title>
  <style>
    body{font-family:system-ui,Segoe UI,Roboto,Arial;max-width:1040px;margin:18px auto;padding:0 12px}
    h1{font-size:20px;margin:8px 0 14px}
    .row{display:flex;flex-wrap:wrap;gap:12px;align-items:stretch}
    .card{border:1px solid #ddd;border-radius:12px;padding:12px;flex:1;min-width:300px}
    .card h2{font-size:14px;margin:0 0 10px;color:#333}
    label{font-size:12px;color:#444;display:block;margin-bottom:4px}
    select,button{font-size:14px;padding:10px;border-radius:10px;border:1px solid #bbb}
    button{cursor:pointer}
    .btns{display:flex;flex-wrap:wrap;gap:10px}
    .ok{color:#0a7}
    .warn{color:#c70}
    pre{background:#fafafa;border:1px solid #eee;padding:10px;border-radius:10px;overflow:auto;min-height:48px}
    .small{font-size:12px;color:#666}
    .top{margin-bottom:12px}
    .toast{
      position:fixed; right:16px; bottom:16px;
      background:#111; color:#fff; padding:12px 14px;
      border-radius:12px; opacity:0; transform:translateY(10px);
      transition:all .25s ease; pointer-events:none;
      max-width:520px; font-size:13px;
    }
    .toast.show{opacity:0.95; transform:translateY(0)}
  </style>
</head>
<body>
  <h1>ESP32 LIS2DW12 Recorder</h1>

  <div class="card top">
    <h2>Info</h2>
    <div id="status" class="small">Loading...</div>
    <div id="fsinfo" class="small">FS: ...</div>
    <pre id="info">...</pre>

    <div class="btns" style="margin-top:10px">
      <button onclick="goUpdate()">FIRMWARE UPDATE</button>
      <button onclick="doReset()">RESET</button>
    </div>
    <div class="small" style="margin-top:8px">
      Firmware update sırasında kayıt/kalibrasyon yapma.
    </div>
  </div>

  <div class="row">
    <div class="card">
      <h2>Settings</h2>

      <label for="hz">Sampling rate (Hz)</label>
      <select id="hz">
        <option value="2">1.6 Hz (LP)</option>
        <option value="13">12.5 Hz</option>
        <option value="25">25 Hz</option>
        <option value="50">50 Hz</option>
        <option value="100" selected>100 Hz</option>
        <option value="200">200 Hz</option>
        <option value="400">400 Hz</option>
        <option value="800">800 Hz</option>
        <option value="1600">1600 Hz</option>
      </select>

      <label for="fs" style="margin-top:10px">G range</label>
      <select id="fs">
        <option value="2" selected>±2 g</option>
        <option value="4">±4 g</option>
        <option value="8">±8 g</option>
        <option value="16">±16 g</option>
      </select>

      <label for="sec" style="margin-top:10px">Record time (s)</label>
      <select id="sec">
        <option value="15">15</option>
        <option value="30">30</option>
        <option value="45">45</option>
        <option value="60" selected>60</option>
        <option value="75">75</option>
        <option value="90">90</option>
        <option value="120">120</option>
        <option value="180">180</option>
      </select>

      <div class="small" style="margin-top:10px">
        Dosya adı browser saatinden alınır: accelYYMMDDHHMMSS.dat
      </div>
    </div>

    <div class="card">
      <h2>Files</h2>

      <label for="fileSel">Select file</label>
      <select id="fileSel"></select>

      <div class="btns" style="margin-top:12px">
        <button onclick="startRec()">START</button>
        <button onclick="stopRec()">STOP</button>
        <button onclick="downloadBin()">DOWNLOAD</button>
        <button onclick="downloadCsv()">DOWNLOAD CSV</button>
        <button onclick="deleteSel()">DELETE</button>
      </div>

      <div class="btns" style="margin-top:12px">
        <button onclick="calibrateStatic()">CALIBRATE (STATIC Z-UP)</button>
        <button onclick="calibrate6()">CALIBRATE (6-POS)</button>
      </div>

      <div class="small" style="margin-top:10px">
        DOWNLOAD / DELETE seçili dosyaya uygulanır.
      </div>
    </div>

    <div class="card">
      <h2>Live (1 Hz)</h2>
      <canvas id="chart" width="420" height="160" style="width:100%;height:160px;border:1px solid #eee;border-radius:10px;background:#fff"></canvas>
      <pre id="live">ax: -, ay: -, az: -</pre>
      <div class="small">Kayıt veya kalibrasyon sırasında live kapalıdır.</div>
    </div>

    <div class="card" style="flex-basis:100%">
      <h2>Analysis (selected file)</h2>

      <div class="btns" style="margin-bottom:10px">
        <button onclick="analyzeSelected()">ANALYZE</button>
        <button onclick="clearAnalysis()">CLEAR</button>
      </div>
        <h2>Frequency Domain (FFT)</h2>
        <select id="fftAxis">
        <option value="x">X</option>
        <option value="y">Y</option>
        <option value="z">Z</option>
        </select>
        <button onclick="runFFT()">FFT</button>

        <canvas id="fftChart" width="980" height="320"
        style="width:100%;height:320px;border:1px solid #eee;border-radius:10px"></canvas>

        <pre id="fftInfo">-</pre>


      <div class="small" id="anaMeta">Select a file and press ANALYZE.</div>

      <div class="row" style="gap:12px;margin-top:10px">
        <div class="card" style="min-width:260px;flex:0.9">
          <h2>Stats (g, calibrated)</h2>
          <pre id="stats">-</pre>
        </div>
        <div class="card" style="min-width:420px;flex:2">
          <h2>Chart</h2>
          <canvas id="bigChart" width="980" height="360"
            style="width:100%;height:360px;border:1px solid #eee;border-radius:10px;background:#fff"></canvas>
          <div class="small">Downsample: ~2000 points max (auto).</div>
        </div>
      </div>
    </div>
  </div>

  <div id="toast" class="toast"></div>

<script>
// (Aşağısı senin V3 JS’in aynısı; sadece reset/update fonksiyonları eklendi.)

// ---- Mini chart (canvas) ----
const CHART_N = 60;
let axBuf = new Array(CHART_N).fill(0);
let ayBuf = new Array(CHART_N).fill(0);
let azBuf = new Array(CHART_N).fill(0);
let bufIdx = 0;
let bufCount = 0;

function pushSample(ax, ay, az){
  axBuf[bufIdx] = ax;
  ayBuf[bufIdx] = ay;
  azBuf[bufIdx] = az;
  bufIdx = (bufIdx + 1) % CHART_N;
  bufCount = Math.min(bufCount + 1, CHART_N);
}

function getBuf(buf, i){
  const start = (bufIdx - bufCount + CHART_N) % CHART_N;
  return buf[(start + i) % CHART_N];
}

function drawChart(){
  const c = document.getElementById("chart");
  if(!c) return;
  const ctx = c.getContext("2d");
  const w = c.width, h = c.height;

  ctx.clearRect(0,0,w,h);

  const mL=36, mR=10, mT=10, mB=18;
  const pw = w - mL - mR;
  const ph = h - mT - mB;

  let ymin = +Infinity, ymax = -Infinity;
  for(let i=0;i<bufCount;i++){
    const ax = getBuf(axBuf,i), ay = getBuf(ayBuf,i), az = getBuf(azBuf,i);
    ymin = Math.min(ymin, ax, ay, az);
    ymax = Math.max(ymax, ax, ay, az);
  }
  if(!isFinite(ymin) || !isFinite(ymax)) { ymin=-1; ymax=1; }
  if (ymax - ymin < 0.1) {
    const mid = (ymax + ymin)/2;
    ymin = mid - 0.05; ymax = mid + 0.05;
  }

  const pad = (ymax - ymin) * 0.12;
  ymin -= pad; ymax += pad;

  const xAt = (i)=> mL + (bufCount<=1 ? 0 : (i/(bufCount-1))*pw);
  const yAt = (v)=> mT + ( (ymax - v) / (ymax - ymin) ) * ph;

  ctx.lineWidth = 1;
  ctx.strokeStyle = "#f0f0f0";
  ctx.fillStyle = "#888";
  ctx.font = "11px system-ui,Segoe UI,Roboto,Arial";

  const gridN = 4;
  for(let g=0; g<=gridN; g++){
    const y = mT + (g/gridN)*ph;
    ctx.beginPath();
    ctx.moveTo(mL, y);
    ctx.lineTo(mL+pw, y);
    ctx.stroke();

    const val = (ymax - (g/gridN)*(ymax-ymin));
    ctx.fillText(val.toFixed(2), 4, y+4);
  }

  ctx.strokeStyle = "#e6e6e6";
  ctx.beginPath();
  ctx.rect(mL, mT, pw, ph);
  ctx.stroke();

  function drawSeries(buf, color){
    if(bufCount < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    for(let i=0;i<bufCount;i++){
      const v = getBuf(buf,i);
      const x = xAt(i);
      const y = yAt(v);
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }

  drawSeries(axBuf, "#d33");
  drawSeries(ayBuf, "#3a3");
  drawSeries(azBuf, "#36c");

  ctx.fillStyle = "#111";
  ctx.fillText("ax", mL+6, mT+12);
  ctx.fillStyle = "#3a3";
  ctx.fillText("ay", mL+34, mT+12);
  ctx.fillStyle = "#36c";
  ctx.fillText("az", mL+62, mT+12);

  ctx.fillStyle = "#666";
  ctx.fillText(`${Math.max(0,bufCount-1)}s window`, w-110, h-5);
}

async function getJson(path){
  const r = await fetch(path, {cache:"no-store"});
  return await r.json();
}
async function getText(path){
  const r = await fetch(path, {cache:"no-store"});
  const t = await r.text();
  return {ok:r.ok, text:t};
}
function esc(s){ return encodeURIComponent(s); }

function toast(msg){
  const t = document.getElementById("toast");
  t.textContent = msg;
  t.classList.add("show");
  setTimeout(()=>t.classList.remove("show"), 2600);
}

function tsYYMMDDHHMMSS(){
  const d = new Date();
  const yy = String(d.getFullYear()).slice(-2);
  const MM = String(d.getMonth()+1).padStart(2,'0');
  const DD = String(d.getDate()).padStart(2,'0');
  const HH = String(d.getHours()).padStart(2,'0');
  const mm = String(d.getMinutes()).padStart(2,'0');
  const ss = String(d.getSeconds()).padStart(2,'0');
  return `${yy}${MM}${DD}${HH}${mm}${ss}`;
}

function prettyName(filePath){
  let n = filePath || "";
  if (n.startsWith("/")) n = n.slice(1);
  const m = n.match(/^accel(\d{12})(?:_\d{2})?\.dat$/);
  if (!m) return n;
  const ts = m[1];
  const yy = ts.slice(0,2);
  const MM = ts.slice(2,4);
  const DD = ts.slice(4,6);
  const HH = ts.slice(6,8);
  const mm = ts.slice(8,10);
  const ss = ts.slice(10,12);
  return `${n}  [${yy}-${MM}-${DD} ${HH}:${mm}:${ss}]`;
}

let lastRecording = null;

async function refreshFiles(selectName=""){
  const files = await getJson("/api/list");
  const sel = document.getElementById("fileSel");
  const current = selectName || sel.value;

  sel.innerHTML = "";
  let keep = "";
  files.sort((a,b)=> (b.name.localeCompare(a.name)));

  for(const f of files){
    const opt = document.createElement("option");
    opt.value = f.name;
    opt.textContent = `${prettyName(f.name)}  (${f.size} B)`;
    sel.appendChild(opt);
    if (f.name === current) keep = current;
  }
  if (keep) sel.value = keep;
}

async function refreshFsInfo(){
  const j = await getJson("/api/fsinfo");
  const el = document.getElementById("fsinfo");
  const fmt = (x)=> x < 1024*1024 ? (x/1024).toFixed(1)+" KB" : (x/1024/1024).toFixed(2)+" MB";
  el.textContent = `FS: used ${fmt(j.used)} / total ${fmt(j.total)} (free ${fmt(j.free)})`;
}

async function refreshInfo(){
  const j = await getJson("/api/info");
  const st = document.getElementById("status");

  let flags = [];
  if (j.calibratingStatic) flags.push("CAL(STATIC)");
  if (j.calibrating6) flags.push("CAL(6POS:"+j.calibPose+")");

  st.innerHTML =
    (j.recording ? "<span class='warn'>RECORDING</span>" : "<span class='ok'>IDLE</span>")
    + " | mode: " + (j.mode || "-")
    + " | currentFile: " + (j.currentFile || "-")
    + " | samples: " + j.samples
    + " | elapsed: " + Math.round(j.elapsedMs/1000) + " s"
    + " | maxBacklog: " + j.maxBacklog
    + (flags.length ? (" | <span class='warn'>" + flags.join(" ") + "</span>") : "");

  document.getElementById("info").textContent = JSON.stringify(j, null, 2);

  if (lastRecording === true && j.recording === false) {
    toast(`DONE: ${j.currentFile}  samples=${j.samples}`);
    await refreshFiles(j.currentFile);
    await refreshFsInfo();
  }
  lastRecording = j.recording;
}

async function refreshLive(){
  const j = await getJson("/api/live");
  if(j.enabled === false) return;

  document.getElementById("live").textContent =
    `ax: ${j.ax.toFixed(3)} g\nay: ${j.ay.toFixed(3)} g\naz: ${j.az.toFixed(3)} g`;

  pushSample(j.ax, j.ay, j.az);
  drawChart();
}

async function startRec(){
  const hz = document.getElementById("hz").value;
  const fs = document.getElementById("fs").value;
  const sec = document.getElementById("sec").value;
  const ts = tsYYMMDDHHMMSS();

  const r = await getText(`/api/start?hz=${esc(hz)}&fs=${esc(fs)}&sec=${esc(sec)}&ts=${esc(ts)}`);
  if(!r.ok) alert(r.text);
  else toast("STARTED");

  await refreshInfo();
  await refreshFiles();
  await refreshFsInfo();
}

async function stopRec(){
  const r = await getText("/api/stop");
  if(!r.ok) alert(r.text);
  else toast("STOP requested");

  await refreshInfo();
}

function downloadBin(){
  const sel = document.getElementById("fileSel").value;
  if(!sel){ alert("No file selected"); return; }
  window.location.href = `/download?file=${esc(sel)}`;
}

function downloadCsv(){
  const sel = document.getElementById("fileSel").value;
  if(!sel){ alert("No file selected"); return; }
  window.location.href = `/download_csv?file=${esc(sel)}`;
}

async function deleteSel(){
  const sel = document.getElementById("fileSel").value;
  if(!sel){ alert("No file selected"); return; }
  const r = await getText(`/api/delete?file=${esc(sel)}`);
  if(!r.ok) alert(r.text);
  else toast(`Deleted: ${sel}`);

  await refreshFiles();
  await refreshFsInfo();
}

async function calibrateStatic(){
  if(!confirm("STATIC CALIBRATION\nDevice still, +Z UP.\nStart?")) return;
  const r = await getText("/api/calibrate_static");
  if(!r.ok) alert(r.text);
  else toast("Static calibration started");
}

async function calibrate6(){
  alert(
`6-POS Calibration Wizard
Follow steps: X+, X-, Y+, Y-, Z+, Z-
Keep device still at each step.
(You can watch calibPose in Info)`
  );
  const r = await getText("/api/calibrate6");
  if(!r.ok) alert(r.text);
  else toast("6-POS calibration started");
}

// -------- NEW: reset & firmware update ----------
function goUpdate(){
  // no need to fetch; open upload page
  window.location.href = "/update";
}
async function doReset(){
  if(!confirm("Device will reboot now.\nContinue?")) return;
  toast("Rebooting...");
  await getText("/api/reset");
  // page will drop; user refresh after reconnect
}

// (Analysis JS burada devam ediyor; senin V3 içeriğini aynen bırakabilirsin.)
// Senin mevcut V3 analysis kodu bu dosyada var sayılıyor.

async function runFFT(){
  const file = document.getElementById("fileSel").value;
  const axis = document.getElementById("fftAxis").value;
  if(!file) return alert("Select file");

  const j = await getJson(`/api/fft?file=${esc(file)}&axis=${axis}`);
  drawFFT(j.fft, j.df);
  document.getElementById("fftInfo").textContent =
    `Axis: ${j.axis}\nPeak: ${j.peak_hz.toFixed(2)} Hz\nMagnitude: ${j.peak_mag.toFixed(4)}`;
}

function drawFFT(arr, df){
    const c = document.getElementById("fftChart");
    const ctx = c.getContext("2d");
    ctx.clearRect(0,0,c.width,c.height);

    const w=c.width,h=c.height;
    const mL=50,mR=10,mT=10,mB=30;
    const pw=w-mL-mR,ph=h-mT-mB;

    let max=Math.max(...arr);
    if(max<=0) max=1;

    ctx.strokeStyle="#ccc";
    ctx.strokeRect(mL,mT,pw,ph);

    ctx.beginPath();
    ctx.strokeStyle="#36c";
    for(let i=0;i<arr.length;i++){
        const x=mL+(i/(arr.length-1))*pw;
        const y=mT+(1-arr[i]/max)*ph;
        if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();

    ctx.fillStyle="#666";
    ctx.fillText(`0 Hz`,mL,h-8);
    ctx.fillText(`${(arr.length*df).toFixed(1)} Hz`,w-80,h-8);
    }


function clearAnalysis(){
  document.getElementById("stats").textContent = "-";
  document.getElementById("anaMeta").textContent = "Select a file and press ANALYZE.";
  const c=document.getElementById("bigChart");
  const ctx=c.getContext("2d");
  ctx.clearRect(0,0,c.width,c.height);
}

function drawBigChart(ax, ay, az, rateHz){
  const c = document.getElementById("bigChart");
  if(!c) return;
  const ctx = c.getContext("2d");

  const w = c.width, h = c.height;
  ctx.clearRect(0,0,w,h);

  const mL=50, mR=10, mT=10, mB=30;
  const pw=w-mL-mR, ph=h-mT-mB;

  let ymin=Infinity, ymax=-Infinity;
  [...ax,...ay,...az].forEach(v=>{
    ymin=Math.min(ymin,v);
    ymax=Math.max(ymax,v);
  });
  if(!isFinite(ymin)||!isFinite(ymax)){ ymin=-1; ymax=1; }
  if(ymax-ymin<0.01){
    const m=(ymax+ymin)/2;
    ymin=m-0.01; ymax=m+0.01;
  }

  const xAt=(i,n)=>mL+(i/(n-1))*pw;
  const yAt=v=>mT+((ymax-v)/(ymax-ymin))*ph;

  // grid
  ctx.strokeStyle="#eee";
  ctx.fillStyle="#666";
  ctx.font="11px system-ui";
  for(let i=0;i<=4;i++){
    const y=mT+(i/4)*ph;
    ctx.beginPath(); ctx.moveTo(mL,y); ctx.lineTo(mL+pw,y); ctx.stroke();
    const val=(ymax-(i/4)*(ymax-ymin));
    ctx.fillText(val.toFixed(3),4,y+4);
  }

  // frame
  ctx.strokeStyle="#ccc";
  ctx.strokeRect(mL,mT,pw,ph);

  function plot(arr,color){
    if(arr.length<2) return;
    ctx.strokeStyle=color;
    ctx.lineWidth=2;
    ctx.beginPath();
    for(let i=0;i<arr.length;i++){
      const x=xAt(i,arr.length);
      const y=yAt(arr[i]);
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }

  plot(ax,"#d33"); // X
  plot(ay,"#2a2"); // Y
  plot(az,"#26c"); // Z

  ctx.fillStyle="#000";
  ctx.fillText("X",mL+10,mT+12);
  ctx.fillStyle="#2a2";
  ctx.fillText("Y",mL+30,mT+12);
  ctx.fillStyle="#26c";
  ctx.fillText("Z",mL+50,mT+12);

  ctx.fillStyle="#666";
  ctx.fillText(`${(ax.length/rateHz).toFixed(2)} s`, w-70, h-8);
}




async function analyzeSelected(){
  const file = document.getElementById("fileSel").value;
  if(!file){ alert("No file selected"); return; }

  try{
    toast("Analyzing on device...");
    document.getElementById("anaMeta").textContent = "Analyzing on device...";

    const j = await getJson(`/api/analyze?file=${esc(file)}`);

    // chart
    drawBigChart(j.ax, j.ay, j.az, j.eff_hz || j.rate_hz || 1);

    // stats text
    const stats =
`File: ${j.file}
rate_hz: ${j.rate_hz}
record_s: ${j.record_s}
samples(header): ${j.samples_header}
samples(used): ${j.samples_used}
fs_g: ±${j.fs_g}g
res_bits: ${j.res_bits}
q_bits: ${j.q_bits}

X: min=${j.min[0].toFixed(4)} g  max=${j.max[0].toFixed(4)} g  rms=${j.rms[0].toFixed(4)} g
Y: min=${j.min[1].toFixed(4)} g  max=${j.max[1].toFixed(4)} g  rms=${j.rms[1].toFixed(4)} g
Z: min=${j.min[2].toFixed(4)} g  max=${j.max[2].toFixed(4)} g  rms=${j.rms[2].toFixed(4)} g`;

    document.getElementById("stats").textContent = stats;
    document.getElementById("anaMeta").textContent =
      `OK. Rendered ${j.pts} points (downsampled on ESP32).`;

    toast("Analysis done.");
  }catch(e){
    console.error(e);
    alert("ANALYZE failed: " + (e?.message || e));
    document.getElementById("anaMeta").textContent = "ANALYZE failed.";
  }
}


setInterval(refreshInfo, 1000);
setInterval(()=>refreshFiles(), 2000);
setInterval(refreshFsInfo, 3000);
setInterval(refreshLive, 1000);

refreshInfo(); refreshFiles(); refreshFsInfo(); refreshLive(); drawChart();
</script>
</body>
</html>
)HTML";

// ======================= API JSON =======================
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
  g_listCache = buildFilesJsonNow();
  g_listCacheMs = millis();

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
    s += "\"ax\":" + String(g_live_g[0], 3) + ",";
    s += "\"ay\":" + String(g_live_g[1], 3) + ",";
    s += "\"az\":" + String(g_live_g[2], 3);
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

  Wire.setClock(400000);
  LIS2DW12 lis(Wire, 0x18);
  if (!lis.begin(-1, -1, 400000))
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
  lis.setRateHz(100);

  lis.loadCalibrationNVS("lis2dw12", "cal");
  g_calDirty = false;

  float g[3] = {0, 0, 0};
  lis.readG(g);

  g_live_g[0] = g[0];
  g_live_g[1] = g[1];
  g_live_g[2] = g[2];

  if (g_i2cMutex)
    xSemaphoreGive(g_i2cMutex);

  String s = "{";
  s += "\"enabled\":true,";
  s += "\"ax\":" + String(g[0], 3) + ",";
  s += "\"ay\":" + String(g[1], 3) + ",";
  s += "\"az\":" + String(g[2], 3);
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

// ======================= WiFi connect with AP fallback =======================
static void startWiFiOrAP()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("WiFi connecting");
  const uint32_t start = millis();
  const uint32_t timeoutMs = 12000;

  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    g_apMode = false;
    g_apSsid = "";
    Serial.print("STA IP: ");
    Serial.println(WiFi.localIP());
    return;
  }

  // fallback AP
  WiFi.disconnect(true);
  delay(100);

  uint64_t mac = ESP.getEfuseMac();
  uint32_t suf = (uint32_t)(mac & 0xFFFFFF);
  char ssid[32];
  snprintf(ssid, sizeof(ssid), "LIS2DW12-%06lX", (unsigned long)suf);

  g_apMode = true;
  g_apSsid = ssid;

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(ssid); // open AP (no password)
  delay(200);

  Serial.println(ok ? "AP started" : "AP start FAIL");
  Serial.print("AP SSID: ");
  Serial.println(ssid);
  Serial.print("AP IP  : ");
  Serial.println(WiFi.softAPIP());
}

// ======================= Setup / loop =======================
void setup()
{
  Serial.begin(115200);
  delay(200);

  if (!LittleFS.begin(true))
  {
    Serial.println("LittleFS mount/format FAIL");
    while (1)
      delay(1000);
  }

  // I2C init once
  Wire.begin(21, 22);
  Wire.setClock(400000);

  g_i2cMutex = xSemaphoreCreateMutex();
  if (!g_i2cMutex)
  {
    Serial.println("I2C mutex alloc FAIL");
    while (1)
      delay(1000);
  }

  // WiFi (STA or AP fallback)
  startWiFiOrAP();

  g_listCache = buildFilesJsonNow();
  g_listCacheMs = millis();

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

  // NEW
  server.on("/api/reset", handleApiReset);
  server.on("/update", HTTP_GET, handleUpdateGet);
  server.on("/update", HTTP_POST, handleUpdatePost, handleUpdateUpload);

  server.on("/api/version", handleApiVersion);
  server.on("/api/analyze", handleApiAnalyze);
  server.on("/api/fft", handleApiFFT);

  server.begin();

  Serial.println("Web ready:");
  if (!g_apMode)
  {
    Serial.println("  http://<STA_IP>/  (try /ping too)");
  }
  else
  {
    Serial.println("  http://192.168.4.1/  (AP mode)");
  }
}

void loop()
{
  server.handleClient();
  delay(1);
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
