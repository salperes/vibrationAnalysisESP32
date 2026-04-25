#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>

#include "trigger.h"
#include "recording.h"      // startTimerHz, stopTimer, consumeTimerDue, resetTimerDue
#include "sensor_utils.h"
#include "file_handlers.h"  // rebuildListCache
#include "app_state.h"

// --- Local sample-rate helper (mirrors recording.cpp's parser) ---
static bool parseHzForTrigger(uint16_t uiHz, uint16_t &outHz, LIS2DW12::Mode &mode)
{
  if (uiHz == 2) { outHz = 2; mode = LIS2DW12::Mode::LowPower;  return true; }
  const uint16_t allowed[] = {13, 25, 50, 100, 200, 400, 800, 1600};
  for (auto v : allowed)
    if (uiHz == v) { outHz = v; mode = LIS2DW12::Mode::HighPerf; return true; }
  return false;
}

// --- Pre-roll ring buffer (heap) ---
struct Ring {
  Sample6 *buf;
  uint32_t cap;
  uint32_t head;     // next write index
  uint32_t filled;   // 0..cap
};
static void ringInit(Ring &r, uint32_t cap) {
  r.buf = (Sample6 *)malloc(cap * sizeof(Sample6));
  r.cap = r.buf ? cap : 0;
  r.head = 0;
  r.filled = 0;
}
static void ringFree(Ring &r) {
  if (r.buf) free(r.buf);
  r.buf = nullptr;
  r.cap = r.head = r.filled = 0;
}
static inline void ringPush(Ring &r, int16_t ax, int16_t ay, int16_t az) {
  r.buf[r.head].ax = ax;
  r.buf[r.head].ay = ay;
  r.buf[r.head].az = az;
  r.head = (r.head + 1) % r.cap;
  if (r.filled < r.cap) r.filled++;
}

// --- 100 ms detection window (collect-then-evaluate, non-overlapping) ---
struct Win {
  float *ax, *ay, *az;
  uint16_t cap;
  uint16_t cnt;
};
static void winInit(Win &w, uint16_t cap) {
  w.ax = (float *)malloc(cap * sizeof(float));
  w.ay = (float *)malloc(cap * sizeof(float));
  w.az = (float *)malloc(cap * sizeof(float));
  w.cap = (w.ax && w.ay && w.az) ? cap : 0;
  w.cnt = 0;
}
static void winFree(Win &w) {
  if (w.ax) free(w.ax);
  if (w.ay) free(w.ay);
  if (w.az) free(w.az);
  w.ax = w.ay = w.az = nullptr;
  w.cap = w.cnt = 0;
}
static inline void winAdd(Win &w, float x, float y, float z) {
  if (w.cnt < w.cap) {
    w.ax[w.cnt] = x;
    w.ay[w.cnt] = y;
    w.az[w.cnt] = z;
    w.cnt++;
  }
}
// 3-axis vector AC RMS magnitude (mean-removed)
static float winAcRms(const Win &w) {
  if (w.cnt == 0) return 0.0f;
  double sx = 0, sy = 0, sz = 0;
  for (uint16_t i = 0; i < w.cnt; i++) { sx += w.ax[i]; sy += w.ay[i]; sz += w.az[i]; }
  const float mx = (float)(sx / w.cnt);
  const float my = (float)(sy / w.cnt);
  const float mz = (float)(sz / w.cnt);
  double ss = 0;
  for (uint16_t i = 0; i < w.cnt; i++) {
    const float dx = w.ax[i] - mx;
    const float dy = w.ay[i] - my;
    const float dz = w.az[i] - mz;
    ss += (double)(dx * dx + dy * dy + dz * dz);
  }
  return sqrtf((float)(ss / (double)w.cnt));
}

// --- Filename helper for grab files ("/grab" + ts + ".dat") ---
static String makeGrabFilename(const String &ts12) {
  String base = "/grab" + ts12;
  String path = base + ".dat";
  if (!LittleFS.exists(path)) return path;
  for (int i = 1; i <= 99; i++) {
    char suf[8];
    snprintf(suf, sizeof(suf), "_%02d", i);
    String p2 = base + String(suf) + ".dat";
    if (!LittleFS.exists(p2)) return p2;
  }
  return String();
}

// --- Trigger task ---------------------------------------------------------
static const uint32_t ARMED_TIMEOUT_MS = 5UL * 60UL * 1000UL; // 5 minutes
static const uint8_t  TRIGGER_CONFIRM_WINDOWS = 3;            // 300 ms above
static const uint8_t  STABLE_CONFIRM_WINDOWS  = 5;            // 500 ms below

struct TrigCtx {
  // Timing-derived constants
  uint16_t hz;
  uint8_t  fs_g;
  LIS2DW12::Mode mode;
  uint32_t preCap;     // pre_s * hz
  uint32_t maxSamples; // max_s * hz (cap)
  uint16_t winN;       // 100 ms window size
};

static void cleanupAndExit(File *f, Ring *ring, Win *win, bool gaveMutex)
{
  if (f && *f) f->close();
  if (ring) ringFree(*ring);
  if (win)  winFree(*win);
  if (gaveMutex) {
    // Already released by caller path; nothing to do.
  }
  stopTimer();
  if (g_i2cMutex) xSemaphoreGive(g_i2cMutex);
  g_trigState = TrigState::Idle;
  g_trigDisarmRequested = false;
  g_trigTask = nullptr;
  vTaskDelete(nullptr);
}

static void triggerTask(void *arg)
{
  TrigCtx ctx = *(TrigCtx *)arg;
  free(arg);

  g_liveSensorReady = false;
  g_trigDisarmRequested = false;
  g_trigState = TrigState::Armed;
  g_trigArmedAtMs = millis();
  g_trigFiredAtMs = 0;
  g_trigPostStartMs = 0;
  g_trigBaseline = 0.0f;
  g_trigCurrentRms = 0.0f;
  g_samplesWritten = 0;
  g_maxBacklog = 0;
  g_elapsedMs = 0;
  g_currentFile = "";

  if (!g_i2cMutex || xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
    Serial.println("[TRIG] I2C mutex timeout");
    g_trigState = TrigState::Idle;
    g_trigTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  Wire.setClock(400000);
  LIS2DW12 lis(Wire, 0x18);
  if (!lis.begin(-1, -1, 400000)) {
    Serial.println("[TRIG] Sensor init failed");
    if (g_i2cMutex) xSemaphoreGive(g_i2cMutex);
    g_trigState = TrigState::Idle;
    g_trigTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  LIS2DW12::Config cfg;
  cfg.mode = ctx.mode;
  cfg.lpMode = (ctx.mode == LIS2DW12::Mode::LowPower)
                   ? LIS2DW12::LowPowerMode::LP1_12bit
                   : LIS2DW12::LowPowerMode::LP2_14bit;
  cfg.fs = fsFromG(ctx.fs_g);
  cfg.lowNoise = true;
  cfg.bdu = true;
  cfg.autoInc = true;
  lis.applyConfig(cfg);
  if (ctx.mode == LIS2DW12::Mode::LowPower && ctx.hz == 2) {
    lis.setPowerMode(LIS2DW12::Odr::Hz12_5_or_1_6, ctx.mode, cfg.lpMode);
  } else {
    lis.setRateHz(ctx.hz);
  }
  lis.loadCalibrationNVS("lis2dw12", "cal");
  auto cal = lis.getCalibration();
  const uint8_t resBits = lis.activeResolutionBits();
  const uint8_t fs_g = fsToByte(cfg.fs);

  // Allocate buffers
  Ring ring{};
  ringInit(ring, ctx.preCap);
  Win win{};
  winInit(win, ctx.winN);
  if (!ring.buf || !win.ax) {
    Serial.println("[TRIG] OOM ring/window");
    cleanupAndExit(nullptr, &ring, &win, false);
    return;
  }

  // Start hardware timer; samples become "due" at ctx.hz rate
  resetTimerDue();
  startTimerHz(ctx.hz);

  // ---- Phase BASELINE: collect 1 second of samples to set threshold ----
  const uint32_t baselineSamples = ctx.hz; // ~1 s
  uint32_t collected = 0;
  while (collected < baselineSamples && !g_trigDisarmRequested) {
    uint32_t due = consumeTimerDue();
    while (due && collected < baselineSamples && !g_trigDisarmRequested) {
      int16_t ax, ay, az;
      if (lis.readRawAligned(ax, ay, az)) {
        ringPush(ring, ax, ay, az);
        const float gx = applyCal1(rawAlignedToG(ax, resBits, fs_g), cal.offset_g[0], cal.scale[0]);
        const float gy = applyCal1(rawAlignedToG(ay, resBits, fs_g), cal.offset_g[1], cal.scale[1]);
        const float gz = applyCal1(rawAlignedToG(az, resBits, fs_g), cal.offset_g[2], cal.scale[2]);
        winAdd(win, gx * GRAVITY_MPS2, gy * GRAVITY_MPS2, gz * GRAVITY_MPS2);
        collected++;
      }
      due--;
    }
    delay(0);
  }
  if (g_trigDisarmRequested) {
    cleanupAndExit(nullptr, &ring, &win, true);
    return;
  }
  // Use the entire collected window for baseline (single-shot, larger N)
  // but evaluate via mean-removed RMS just like detection windows.
  g_trigBaseline = winAcRms(win);
  win.cnt = 0; // reset for sliding evaluation
  if (g_trigMode == 0) {
    g_trigEffThreshold = g_trigBaseline * (float)g_trigMult;
    if (g_trigEffThreshold <= 0.0f) g_trigEffThreshold = 0.01f; // safety floor
  } else {
    g_trigEffThreshold = g_trigManualThr;
  }
  Serial.printf("[TRIG] baseline=%.4f m/s^2  threshold=%.4f m/s^2\n",
                g_trigBaseline, g_trigEffThreshold);

  // ---- Phase ARMED: detect onset of motion ----
  uint8_t consecAbove = 0;
  while (g_trigState == TrigState::Armed && !g_trigDisarmRequested) {
    if ((millis() - g_trigArmedAtMs) > ARMED_TIMEOUT_MS) {
      Serial.println("[TRIG] Armed timeout");
      g_trigDisarmRequested = true;
      break;
    }
    uint32_t due = consumeTimerDue();
    while (due) {
      int16_t ax, ay, az;
      if (lis.readRawAligned(ax, ay, az)) {
        ringPush(ring, ax, ay, az);
        const float gx = applyCal1(rawAlignedToG(ax, resBits, fs_g), cal.offset_g[0], cal.scale[0]);
        const float gy = applyCal1(rawAlignedToG(ay, resBits, fs_g), cal.offset_g[1], cal.scale[1]);
        const float gz = applyCal1(rawAlignedToG(az, resBits, fs_g), cal.offset_g[2], cal.scale[2]);
        winAdd(win, gx * GRAVITY_MPS2, gy * GRAVITY_MPS2, gz * GRAVITY_MPS2);
        if (win.cnt >= win.cap) {
          const float rms = winAcRms(win);
          win.cnt = 0;
          g_trigCurrentRms = rms;
          if (rms >= g_trigEffThreshold) {
            consecAbove++;
            if (consecAbove >= TRIGGER_CONFIRM_WINDOWS) {
              g_trigState = TrigState::Triggered;
              g_trigFiredAtMs = millis();
              break;
            }
          } else {
            consecAbove = 0;
          }
        }
      }
      due--;
    }
    delay(0);
  }
  if (g_trigDisarmRequested) {
    cleanupAndExit(nullptr, &ring, &win, true);
    return;
  }

  // ---- Transition to TRIGGERED: open file, dump pre-roll ----
  String path = makeGrabFilename(g_uiTimestamp);
  if (path.length() == 0) {
    Serial.println("[TRIG] Unique filename failed");
    cleanupAndExit(nullptr, &ring, &win, true);
    return;
  }
  g_currentFile = path;

  File f = LittleFS.open(path, "w");
  if (!f) {
    Serial.println("[TRIG] Open failed");
    cleanupAndExit(nullptr, &ring, &win, true);
    return;
  }

  FileHeaderV4 h{};
  memcpy(h.magic, "LIS2DW12", 8);
  h.version  = 4;
  h.rate_hz  = ctx.hz;
  h.record_s = g_trigMaxS;
  h.samples  = 0;
  h.fs_g     = fs_g;
  h.res_bits = resBits;
  h.q_bits   = 0;
  h.flags    = 0x01; // triggered
  for (int i = 0; i < 3; i++) {
    h.cal_offset_g[i] = cal.offset_g[i];
    h.cal_scale[i]    = cal.scale[i];
  }
  h.pre_samples    = ring.filled;
  h.threshold_used = g_trigEffThreshold;
  h.trig_mode      = g_trigMode;
  h.trig_mult      = (g_trigMode == 0) ? g_trigMult : 0;
  memcpy(h.serial_no,     g_device_serial,         sizeof(h.serial_no));
  memcpy(h.device_name,   g_device_name,           sizeof(h.device_name));
  memcpy(h.meas_point,    g_recMeta.meas_point,    sizeof(h.meas_point));
  memcpy(h.scan_dir,      g_recMeta.scan_dir,      sizeof(h.scan_dir));
  memcpy(h.operator_name, g_recMeta.operator_name, sizeof(h.operator_name));
  memcpy(h.notes,         g_recMeta.notes,         sizeof(h.notes));
  if (f.write((uint8_t *)&h, sizeof(h)) != sizeof(h)) {
    Serial.println("[TRIG] Header write failed");
    cleanupAndExit(&f, &ring, &win, true);
    return;
  }

  // Dump pre-roll in chronological order
  const uint32_t preStart = (ring.filled < ring.cap) ? 0 : ring.head;
  uint32_t preWritten = 0;
  for (uint32_t i = 0; i < ring.filled; i++) {
    Sample6 s = ring.buf[(preStart + i) % ring.cap];
    if (f.write((uint8_t *)&s, sizeof(s)) == sizeof(s)) preWritten++;
    if ((i & 0xFF) == 0) delay(0);
  }
  g_samplesWritten = preWritten;
  ringFree(ring); // free pre-roll memory; from now on we stream live samples

  // ---- Phase TRIGGERED: stream samples until stable or max_s ----
  const size_t CHUNK_N = 1024;
  Sample6 *chunk = (Sample6 *)malloc(CHUNK_N * sizeof(Sample6));
  if (!chunk) {
    Serial.println("[TRIG] OOM chunk");
    rewriteHeaderSamples(path, g_samplesWritten);
    cleanupAndExit(&f, nullptr, &win, true);
    return;
  }
  size_t chunkFill = 0;
  uint8_t consecBelow = 0;
  bool truncated = false;

  while (g_trigState == TrigState::Triggered && !g_trigDisarmRequested) {
    if (g_samplesWritten >= ctx.maxSamples) {
      truncated = true;
      g_trigState = TrigState::PostTail;
      g_trigPostStartMs = millis();
      break;
    }
    uint32_t due = consumeTimerDue();
    while (due) {
      int16_t ax, ay, az;
      if (lis.readRawAligned(ax, ay, az)) {
        chunk[chunkFill].ax = ax;
        chunk[chunkFill].ay = ay;
        chunk[chunkFill].az = az;
        chunkFill++;
        const float gx = applyCal1(rawAlignedToG(ax, resBits, fs_g), cal.offset_g[0], cal.scale[0]);
        const float gy = applyCal1(rawAlignedToG(ay, resBits, fs_g), cal.offset_g[1], cal.scale[1]);
        const float gz = applyCal1(rawAlignedToG(az, resBits, fs_g), cal.offset_g[2], cal.scale[2]);
        winAdd(win, gx * GRAVITY_MPS2, gy * GRAVITY_MPS2, gz * GRAVITY_MPS2);
        if (win.cnt >= win.cap) {
          const float rms = winAcRms(win);
          win.cnt = 0;
          g_trigCurrentRms = rms;
          if (rms < g_trigEffThreshold) {
            consecBelow++;
            if (consecBelow >= STABLE_CONFIRM_WINDOWS) {
              g_trigState = TrigState::PostTail;
              g_trigPostStartMs = millis();
              break;
            }
          } else {
            consecBelow = 0;
          }
        }

        if (chunkFill >= CHUNK_N) {
          size_t bytes = chunkFill * sizeof(Sample6);
          if (f.write((uint8_t *)chunk, bytes) == bytes) {
            g_samplesWritten += chunkFill;
          }
          chunkFill = 0;
        }
      }
      due--;
    }
    delay(0);
  }

  // ---- Phase POST_TAIL: keep recording for post_s seconds ----
  while (g_trigState == TrigState::PostTail && !g_trigDisarmRequested) {
    if ((millis() - g_trigPostStartMs) >= (uint32_t)g_trigPostS * 1000UL) break;
    if (g_samplesWritten >= ctx.maxSamples + (uint32_t)g_trigPostS * ctx.hz) {
      truncated = true;
      break;
    }
    uint32_t due = consumeTimerDue();
    while (due) {
      int16_t ax, ay, az;
      if (lis.readRawAligned(ax, ay, az)) {
        chunk[chunkFill].ax = ax;
        chunk[chunkFill].ay = ay;
        chunk[chunkFill].az = az;
        chunkFill++;
        if (chunkFill >= CHUNK_N) {
          size_t bytes = chunkFill * sizeof(Sample6);
          if (f.write((uint8_t *)chunk, bytes) == bytes) {
            g_samplesWritten += chunkFill;
          }
          chunkFill = 0;
        }
      }
      due--;
    }
    delay(0);
  }

  // Flush remaining chunk
  if (chunkFill > 0) {
    size_t bytes = chunkFill * sizeof(Sample6);
    if (f.write((uint8_t *)chunk, bytes) == bytes) {
      g_samplesWritten += chunkFill;
    }
  }
  free(chunk);
  stopTimer();

  // Finalize header (samples count + truncated flag if any)
  f.close();
  rewriteHeaderSamples(path, g_samplesWritten);
  if (truncated) {
    File ff = LittleFS.open(path, "r+");
    if (ff) {
      // flags is at offset 21 in V4; read+modify+write that single byte.
      ff.seek(21, SeekSet);
      uint8_t flags = 0;
      if (ff.read(&flags, 1) == 1) {
        flags |= 0x02;
        ff.seek(21, SeekSet);
        ff.write(&flags, 1);
      }
      ff.close();
    }
  }
  rebuildListCache();

  winFree(win);
  if (g_i2cMutex) xSemaphoreGive(g_i2cMutex);
  g_trigState = TrigState::Idle;
  g_trigDisarmRequested = false;
  g_trigTask = nullptr;
  vTaskDelete(nullptr);
}

// --- HTTP handlers --------------------------------------------------------
static bool isAnyBusy()
{
  return g_recording || g_calibratingStatic || g_calibrating6 ||
         g_trigState != TrigState::Idle;
}

void handleApiTriggerArm()
{
  if (isAnyBusy()) {
    server.send(409, "text/plain", "Busy");
    return;
  }

  // ---- Acquisition params (same allowed sets as manual /api/start) ----
  uint16_t uiHz = server.hasArg("hz") ? (uint16_t)server.arg("hz").toInt() : 800;
  uint8_t  fs_g = server.hasArg("fs") ? (uint8_t)server.arg("fs").toInt() : 2;

  String ts = server.hasArg("ts") ? server.arg("ts") : "";
  if (!isValidYYMMDDHHMMSS(ts)) {
    server.send(400, "text/plain", "Invalid ts (need YYMMDDHHMMSS from browser)");
    return;
  }

  uint16_t hz = 800;
  LIS2DW12::Mode mode = LIS2DW12::Mode::HighPerf;
  if (!parseHzForTrigger(uiHz, hz, mode)) {
    server.send(400, "text/plain", "Invalid hz");
    return;
  }
  if (!(fs_g == 2 || fs_g == 4 || fs_g == 8 || fs_g == 16)) {
    server.send(400, "text/plain", "Invalid fs");
    return;
  }

  // ---- Trigger params ----
  uint16_t pre_s  = server.hasArg("pre_s")  ? (uint16_t)server.arg("pre_s").toInt()  : 3;
  uint16_t post_s = server.hasArg("post_s") ? (uint16_t)server.arg("post_s").toInt() : 5;
  uint16_t max_s  = server.hasArg("max_s")  ? (uint16_t)server.arg("max_s").toInt()  : 60;
  if (pre_s == 0  || pre_s  > 10) { server.send(400, "text/plain", "pre_s 1..10");  return; }
  if (post_s == 0 || post_s > 30) { server.send(400, "text/plain", "post_s 1..30"); return; }
  if (max_s < 5   || max_s  > 120){ server.send(400, "text/plain", "max_s 5..120"); return; }

  uint8_t trigMode = server.hasArg("mode") ? (uint8_t)server.arg("mode").toInt() : 0;
  uint8_t trigMult = server.hasArg("mult") ? (uint8_t)server.arg("mult").toInt() : 3;
  float manualThr  = server.hasArg("abs_thr") ? server.arg("abs_thr").toFloat() : 0.5f;
  if (trigMode > 1) { server.send(400, "text/plain", "mode 0|1"); return; }
  if (trigMode == 0 && (trigMult < 2 || trigMult > 20)) {
    server.send(400, "text/plain", "mult 2..20");
    return;
  }
  if (trigMode == 1 && (manualThr <= 0.0f || manualThr > 100.0f)) {
    server.send(400, "text/plain", "abs_thr (0..100] m/s^2");
    return;
  }

  // ---- Metadata strings (truncated to fixed-buffer caps) ----
  copyToFixed(server.arg("meas_point"), g_recMeta.meas_point, sizeof(g_recMeta.meas_point));
  copyToFixed(server.arg("scan_dir"),   g_recMeta.scan_dir,   sizeof(g_recMeta.scan_dir));
  copyToFixed(server.arg("operator"),   g_recMeta.operator_name, sizeof(g_recMeta.operator_name));
  copyToFixed(server.arg("notes"),      g_recMeta.notes,      sizeof(g_recMeta.notes));

  // Pre-flight disk check: header + (pre + max + post) * hz samples + safety
  const size_t totalS = (size_t)pre_s + (size_t)max_s + (size_t)post_s;
  const size_t needed = sizeof(FileHeaderV4) + totalS * (size_t)hz * sizeof(Sample6);
  if (!hasFreeSpaceFor(needed))
  {
    server.send(507, "text/plain", "Insufficient storage for this recording");
    return;
  }

  // Latch trigger config globals
  g_trigPreS = pre_s;
  g_trigPostS = post_s;
  g_trigMaxS = max_s;
  g_trigMode = trigMode;
  g_trigMult = trigMult;
  g_trigManualThr = manualThr;
  g_uiTimestamp = ts;
  g_cfg.hz = hz;
  g_cfg.fs_g = fs_g;
  g_cfg.sec = max_s;
  g_cfg.mode = mode;

  // Build context for task
  TrigCtx *ctx = (TrigCtx *)malloc(sizeof(TrigCtx));
  if (!ctx) { server.send(500, "text/plain", "OOM"); return; }
  ctx->hz   = hz;
  ctx->fs_g = fs_g;
  ctx->mode = mode;
  ctx->preCap     = (uint32_t)pre_s * (uint32_t)hz;
  ctx->maxSamples = (uint32_t)max_s * (uint32_t)hz;
  ctx->winN       = max((uint16_t)8, (uint16_t)(hz / 10)); // 100 ms (>=8)

  g_trigState = TrigState::Armed; // set BEFORE task creation to avoid TOCTOU
  BaseType_t ok = xTaskCreatePinnedToCore(triggerTask, "trig", 8192, ctx, 2, &g_trigTask, 1);
  if (ok != pdPASS) {
    free(ctx);
    g_trigState = TrigState::Idle;
    server.send(500, "text/plain", "Task create failed");
    return;
  }
  server.send(200, "text/plain", "OK ARMED");
}

void handleApiTriggerDisarm()
{
  if (g_trigState == TrigState::Idle) {
    server.send(200, "text/plain", "Already idle");
    return;
  }
  g_trigDisarmRequested = true;
  server.send(200, "text/plain", "OK disarm requested");
}
