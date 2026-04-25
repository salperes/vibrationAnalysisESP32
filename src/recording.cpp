#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>

#include "recording.h"
#include "sensor_utils.h"
#include "file_handlers.h"

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

// ======================= Recording task =======================
static void recordTask(void * /*arg*/)
{
  g_liveSensorReady = false;
  g_recording = true;
  g_stopRequested = false;
  g_samplesWritten = 0;
  g_maxBacklog = 0;
  g_elapsedMs = 0;

  String ts = g_uiTimestamp;
  String path = makeNewFileNameFromUI(ts);
  if (path.length() == 0)
  {
    Serial.println("[REC] Could not allocate unique filename");
    g_recording = false;
    g_recTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }
  g_currentFile = path;

  if (g_i2cMutex)
  {
    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(5000)) != pdTRUE)
    {
      Serial.println("[REC] I2C mutex timeout");
      g_recording = false;
      g_recTask = nullptr;
      vTaskDelete(nullptr);
      return;
    }
  }

  Wire.setClock(400000);
  LIS2DW12 lis(Wire, 0x18);
  if (!lis.begin(-1, -1, 400000))
  {
    Serial.println("[REC] Sensor init failed");
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
  if (!lis.loadCalibrationNVS("lis2dw12", "cal"))
  {
    Serial.println("[REC] No calibration in NVS; recording uncalibrated");
  }
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

  FileHeaderV4 h{};
  memcpy(h.magic, "LIS2DW12", 8);
  h.version  = 4;
  h.rate_hz  = g_cfg.hz;
  h.record_s = g_cfg.sec;
  h.samples  = 0;
  h.fs_g     = fsToByte(cfg.fs);
  h.res_bits = lis.activeResolutionBits();
  h.q_bits   = g_cfg.qBits;
  h.flags    = 0; // manual recording: triggered=0, truncated=0
  for (int i = 0; i < 3; i++)
  {
    h.cal_offset_g[i] = cal.offset_g[i];
    h.cal_scale[i]    = cal.scale[i];
  }
  // Trigger info: zero (manual recording). trig_mode=0xFF marks N/A.
  h.pre_samples    = 0;
  h.threshold_used = 0.0f;
  h.trig_mode      = 0xFF;
  h.trig_mult      = 0;
  // Metadata: copy globals (already null-padded).
  memcpy(h.serial_no,     g_device_serial,    sizeof(h.serial_no));
  memcpy(h.device_name,   g_device_name,      sizeof(h.device_name));
  memcpy(h.meas_point,    g_recMeta.meas_point,    sizeof(h.meas_point));
  memcpy(h.scan_dir,      g_recMeta.scan_dir,      sizeof(h.scan_dir));
  memcpy(h.operator_name, g_recMeta.operator_name, sizeof(h.operator_name));
  memcpy(h.notes,         g_recMeta.notes,         sizeof(h.notes));

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
        size_t bytes = fill * sizeof(Sample6);
        size_t wrote = f.write((uint8_t *)chunk, bytes);
        if (wrote != bytes)
          break;
        g_samplesWritten = idx;
      }
    }

    g_elapsedMs = millis() - tStart;
    delay(0);
  }

  f.close();
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

// ======================= Hz parser =======================
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

// ======================= HTTP Handlers =======================
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

  const uint16_t allowedSec[] = {15, 30, 45, 60, 75, 90, 120};
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
  g_recording = true; // set before task creation to prevent TOCTOU race
  BaseType_t ok = xTaskCreatePinnedToCore(recordTask, "rec", 8192, nullptr, 2, &g_recTask, 1);
  if (ok != pdPASS)
  {
    g_recording = false;
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
