#include <Arduino.h>
#include <Wire.h>

#include "live_preview.h"
#include "sensor_utils.h"
#include "file_handlers.h"

// ======================= Noise measurement =======================
// Computes the AC RMS of the 3-axis acceleration vector magnitude
// after subtracting the per-axis mean (DC / gravity tilt component).
// This is the "still-on-surface" baseline noise floor for the
// realtime ISO 20816 path.
static float measureRealtimeNoise(LIS2DW12 &lis, const uint8_t resBits, const uint8_t fs_g)
{
  const uint16_t samples = 400; // ~0.5s @800 Hz

  float *bx = (float *)malloc(samples * sizeof(float));
  float *by = (float *)malloc(samples * sizeof(float));
  float *bz = (float *)malloc(samples * sizeof(float));
  if (!bx || !by || !bz)
  {
    free(bx);
    free(by);
    free(bz);
    return 0.0f;
  }

  auto cal = lis.getCalibration();
  uint16_t valid = 0;

  for (uint16_t i = 0; i < samples; i++)
  {
    int16_t axRaw, ayRaw, azRaw;
    if (!lis.readRawAligned(axRaw, ayRaw, azRaw))
    {
      delayMicroseconds(1200);
      continue;
    }

    if (g_rawMaskBits)
    {
      int16_t mask = ~((1 << g_rawMaskBits) - 1);
      axRaw &= mask;
      ayRaw &= mask;
      azRaw &= mask;
    }

    float gx = rawAlignedToG(axRaw, resBits, fs_g);
    float gy = rawAlignedToG(ayRaw, resBits, fs_g);
    float gz = rawAlignedToG(azRaw, resBits, fs_g);
    gx = applyCal1(gx, cal.offset_g[0], cal.scale[0]);
    gy = applyCal1(gy, cal.offset_g[1], cal.scale[1]);
    gz = applyCal1(gz, cal.offset_g[2], cal.scale[2]);

    bx[valid] = gx * GRAVITY_MPS2;
    by[valid] = gy * GRAVITY_MPS2;
    bz[valid] = gz * GRAVITY_MPS2;
    valid++;
    delayMicroseconds(1200);
  }

  if (!valid)
  {
    free(bx);
    free(by);
    free(bz);
    return 0.0f;
  }

  double sx = 0, sy = 0, sz = 0;
  for (uint16_t i = 0; i < valid; i++)
  {
    sx += bx[i];
    sy += by[i];
    sz += bz[i];
  }
  const float mx = (float)(sx / (double)valid);
  const float my = (float)(sy / (double)valid);
  const float mz = (float)(sz / (double)valid);

  double sumSq = 0;
  for (uint16_t i = 0; i < valid; i++)
  {
    const float ax = bx[i] - mx;
    const float ay = by[i] - my;
    const float az = bz[i] - mz;
    const float m2 = ax * ax + ay * ay + az * az;
    sumSq += (double)m2;
  }

  free(bx);
  free(by);
  free(bz);

  return sqrtf((float)(sumSq / (double)valid));
}

// ======================= Live config (Hz / fs) =======================
void handleApiLiveConfig()
{
  if (g_recording || g_calibratingStatic || g_calibrating6 || g_trigState != TrigState::Idle)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }
  if (!server.hasArg("hz") && !server.hasArg("fs"))
  {
    server.send(400, "text/plain", "Need hz or fs");
    return;
  }

  uint16_t hz = g_live_pref_hz;
  uint8_t  fs = g_live_pref_fs_g;

  if (server.hasArg("hz"))
  {
    uint16_t req = (uint16_t)server.arg("hz").toInt();
    const uint16_t allowed[] = {100, 200, 400, 800, 1600};
    bool ok = false;
    for (auto v : allowed) if (req == v) { ok = true; break; }
    if (!ok) { server.send(400, "text/plain", "Invalid hz"); return; }
    hz = req;
  }
  if (server.hasArg("fs"))
  {
    uint8_t req = (uint8_t)server.arg("fs").toInt();
    if (!(req == 2 || req == 4 || req == 8 || req == 16))
    {
      server.send(400, "text/plain", "Invalid fs");
      return;
    }
    fs = req;
  }

  const bool changed = (hz != g_live_pref_hz) || (fs != g_live_pref_fs_g);
  g_live_pref_hz   = hz;
  g_live_pref_fs_g = fs;
  if (changed)
  {
    g_calDirty = true;       // force g_liveSensor reinit next /api/live
    g_liveLastMs = 0;        // bypass 1 s cache
    g_realtimeEnabled = false; // cancel realtime mode (cached noise floor invalid)
    resetLivePreviewState();
  }

  String s = "{\"hz\":" + String(hz) + ",\"fs\":" + String(fs) + "}";
  server.send(200, "application/json", s);
}

// ======================= Realtime config =======================
void handleApiRealtimeConfig()
{
  if (g_recording || g_calibratingStatic || g_calibrating6 || g_trigState != TrigState::Idle)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }

  const bool enable = server.hasArg("enable") ? (server.arg("enable") != "0") : false;
  if (server.hasArg("mask"))
  {
    uint8_t bits = (uint8_t)server.arg("mask").toInt();
    setRawMaskBits(bits);
  }
  g_realtimeEnabled = enable;
  g_rt_hist_count = 0;
  g_rt_hist_idx = 0;
  g_rt_noise_mps2 = 0;
  g_liveLastMs = 0;
  resetLivePreviewState();

  if (!enable)
  {
    server.send(200, "text/plain", "Realtime disabled");
    return;
  }

  if (g_i2cMutex)
  {
    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(5000)) != pdTRUE)
    {
      server.send(500, "text/plain", "I2C mutex timeout");
      return;
    }
  }

  Wire.setClock(400000);
  LIS2DW12 lis(Wire, 0x18);
  if (!lis.begin(-1, -1, 400000))
  {
    if (g_i2cMutex)
      xSemaphoreGive(g_i2cMutex);
    server.send(500, "text/plain", "Sensor init failed");
    return;
  }

  LIS2DW12::Config cfg;
  cfg.mode = LIS2DW12::Mode::HighPerf;
  cfg.lpMode = LIS2DW12::LowPowerMode::LP2_14bit;
  cfg.fs = fsFromG(g_live_pref_fs_g);
  cfg.lowNoise = true;
  cfg.bdu = true;
  cfg.autoInc = true;
  lis.applyConfig(cfg);
  lis.setRateHz(g_live_pref_hz);
  lis.loadCalibrationNVS("lis2dw12", "cal");
  g_calDirty = false;

  g_rt_noise_mps2 = measureRealtimeNoise(lis, lis.activeResolutionBits(), fsToByte(cfg.fs));

  if (g_i2cMutex)
    xSemaphoreGive(g_i2cMutex);

  server.send(200, "text/plain", "Realtime enabled");
}

// ======================= Live preview =======================
// All ax/ay/az/vx/vy/vz/dx/dy/dz fields are AC RMS values (mean-removed, then
// LPF, then integrated for vel/disp). dt_us is the measured per-sample period
// for this window; eff_hz = 1e6 / dt_us.
void handleApiLive()
{
  if (g_recording || g_calibratingStatic || g_calibrating6 || g_trigState != TrigState::Idle)
  {
    server.send(200, "application/json", "{\"enabled\":false}");
    return;
  }

  uint32_t now = millis();
  if (!g_realtimeEnabled && now - g_liveLastMs < 1000)
  {
    const float effHz = (g_live_dt_us > 0) ? (1e6f / (float)g_live_dt_us) : 0.0f;
    String s = "{";
    s += "\"enabled\":true,";
    s += "\"hz\":" + String(g_live_pref_hz) + ",";
    s += "\"dt_us\":" + String(g_live_dt_us) + ",";
    s += "\"eff_hz\":" + String(effHz, 1) + ",";
    s += "\"fc\":" + String(g_live_lp_cut_hz, 1) + ",";
    s += "\"ax\":" + String(g_live_acc_mps2[0], 4) + ",";
    s += "\"ay\":" + String(g_live_acc_mps2[1], 4) + ",";
    s += "\"az\":" + String(g_live_acc_mps2[2], 4) + ",";
    s += "\"mag\":" + String(g_live_mag_acc, 4) + ",";
    s += "\"vx_mmps\":" + String(g_live_vel_mmps[0], 3) + ",";
    s += "\"vy_mmps\":" + String(g_live_vel_mmps[1], 3) + ",";
    s += "\"vz_mmps\":" + String(g_live_vel_mmps[2], 3) + ",";
    s += "\"vmag_mmps\":" + String(g_live_mag_vel_mmps, 3) + ",";
    s += "\"dx_mm\":" + String(g_live_disp_mm[0], 4) + ",";
    s += "\"dy_mm\":" + String(g_live_disp_mm[1], 4) + ",";
    s += "\"dz_mm\":" + String(g_live_disp_mm[2], 4) + ",";
    s += "\"dmag_mm\":" + String(g_live_mag_disp_mm, 4);
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

  if (!g_liveSensorReady || g_calDirty)
  {
    if (!g_liveSensor.begin(-1, -1, 400000))
    {
      if (g_i2cMutex)
        xSemaphoreGive(g_i2cMutex);
      server.send(200, "application/json", "{\"enabled\":false}");
      return;
    }

    LIS2DW12::Config cfg;
    cfg.mode = LIS2DW12::Mode::HighPerf;
    cfg.lpMode = LIS2DW12::LowPowerMode::LP2_14bit;
    cfg.fs = fsFromG(g_live_pref_fs_g);
    cfg.lowNoise = true;
    cfg.bdu = true;
    cfg.autoInc = true;
    g_liveSensor.applyConfig(cfg);
    g_liveSensor.setRateHz(g_live_pref_hz);
    g_liveSensor.loadCalibrationNVS("lis2dw12", "cal");
    g_calDirty = false;
    g_liveSensorReady = true;
  }

  auto cal = g_liveSensor.getCalibration();
  const uint8_t resBits = g_liveSensor.activeResolutionBits();
  const uint8_t fs_g = g_live_pref_fs_g;

  float cutoff = g_live_lp_cut_hz;
  if (server.hasArg("fc"))
  {
    float fcReq = server.arg("fc").toFloat();
    if (fcReq >= 5.0f && fcReq <= (float)g_live_pref_hz * 0.5f)
      cutoff = fcReq;
  }
  g_live_lp_cut_hz = cutoff;

  const uint16_t N = min((uint16_t)200, g_live_pref_hz);

  // Heap-allocated per-axis buffers (3 * N * 4 bytes ~= 2.4 KB at N=200).
  // Acquiring all samples first lets us compute the per-axis mean (DC removal)
  // before integration, which is essential to suppress gravity-tilt drift in
  // the velocity/displacement integrals.
  float *bx = (float *)malloc(N * sizeof(float));
  float *by = (float *)malloc(N * sizeof(float));
  float *bz = (float *)malloc(N * sizeof(float));
  if (!bx || !by || !bz)
  {
    free(bx);
    free(by);
    free(bz);
    if (g_i2cMutex)
      xSemaphoreGive(g_i2cMutex);
    server.send(500, "application/json", "{\"enabled\":false,\"err\":\"oom\"}");
    return;
  }

  uint16_t valid = 0;
  uint32_t t_start = micros();
  for (uint16_t i = 0; i < N; i++)
  {
    int16_t axRaw, ayRaw, azRaw;
    if (!g_liveSensor.readRawAligned(axRaw, ayRaw, azRaw))
    {
      delayMicroseconds(1200);
      continue;
    }
    if (g_rawMaskBits)
    {
      int16_t mask = ~((1 << g_rawMaskBits) - 1);
      axRaw &= mask;
      ayRaw &= mask;
      azRaw &= mask;
    }

    float gx = rawAlignedToG(axRaw, resBits, fs_g);
    float gy = rawAlignedToG(ayRaw, resBits, fs_g);
    float gz = rawAlignedToG(azRaw, resBits, fs_g);
    gx = applyCal1(gx, cal.offset_g[0], cal.scale[0]);
    gy = applyCal1(gy, cal.offset_g[1], cal.scale[1]);
    gz = applyCal1(gz, cal.offset_g[2], cal.scale[2]);

    bx[valid] = gx * GRAVITY_MPS2;
    by[valid] = gy * GRAVITY_MPS2;
    bz[valid] = gz * GRAVITY_MPS2;
    valid++;
    delayMicroseconds(1200);
  }
  uint32_t t_end = micros();

  if (g_i2cMutex)
    xSemaphoreGive(g_i2cMutex);

  if (!valid)
  {
    free(bx);
    free(by);
    free(bz);
    g_liveLastMs = 0;
    server.send(200, "application/json", "{\"enabled\":false}");
    return;
  }

  // Measured dt over the actual sampling window (loop overhead included).
  const float dt = (float)(t_end - t_start) * 1e-6f / (float)valid;
  g_live_dt_us = (uint32_t)((t_end - t_start) / valid);

  // Per-axis mean (DC component: gravity tilt + sensor offset residual).
  double sx = 0, sy = 0, sz = 0;
  for (uint16_t i = 0; i < valid; i++)
  {
    sx += bx[i];
    sy += by[i];
    sz += bz[i];
  }
  const float mx = (float)(sx / (double)valid);
  const float my = (float)(sy / (double)valid);
  const float mz = (float)(sz / (double)valid);

  // First-order LPF on the mean-removed signal (cutoff = `cutoff` Hz).
  const float tau = 1.0f / (2.0f * PI * cutoff);
  const float alpha = dt / (tau + dt);

  if (!g_realtimeEnabled)
  {
    // Per-axis: AC -> LPF -> integrate vel/disp -> RMS
    float lpf[3] = {0, 0, 0};
    bool lpfInit = false;
    float vel[3] = {0, 0, 0};
    float disp[3] = {0, 0, 0};
    double ss_a[3] = {0, 0, 0};
    double ss_v[3] = {0, 0, 0};
    double ss_d[3] = {0, 0, 0};

    for (uint16_t i = 0; i < valid; i++)
    {
      const float a[3] = {bx[i] - mx, by[i] - my, bz[i] - mz};
      if (!lpfInit)
      {
        lpf[0] = a[0];
        lpf[1] = a[1];
        lpf[2] = a[2];
        lpfInit = true;
      }
      else
      {
        lpf[0] += alpha * (a[0] - lpf[0]);
        lpf[1] += alpha * (a[1] - lpf[1]);
        lpf[2] += alpha * (a[2] - lpf[2]);
      }
      vel[0] += lpf[0] * dt;
      vel[1] += lpf[1] * dt;
      vel[2] += lpf[2] * dt;
      disp[0] += vel[0] * dt;
      disp[1] += vel[1] * dt;
      disp[2] += vel[2] * dt;

      ss_a[0] += (double)lpf[0] * lpf[0];
      ss_a[1] += (double)lpf[1] * lpf[1];
      ss_a[2] += (double)lpf[2] * lpf[2];
      ss_v[0] += (double)vel[0] * vel[0];
      ss_v[1] += (double)vel[1] * vel[1];
      ss_v[2] += (double)vel[2] * vel[2];
      ss_d[0] += (double)disp[0] * disp[0];
      ss_d[1] += (double)disp[1] * disp[1];
      ss_d[2] += (double)disp[2] * disp[2];
    }

    free(bx);
    free(by);
    free(bz);

    const float invN = 1.0f / (float)valid;
    for (int k = 0; k < 3; k++)
    {
      g_live_acc_mps2[k] = sqrtf((float)(ss_a[k] * invN));
      g_live_vel_mmps[k] = sqrtf((float)(ss_v[k] * invN)) * 1000.0f;
      g_live_disp_mm[k]  = sqrtf((float)(ss_d[k] * invN)) * 1000.0f;
      g_live_g[k]        = g_live_acc_mps2[k] / GRAVITY_MPS2;
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

    const float effHz = (g_live_dt_us > 0) ? (1e6f / (float)g_live_dt_us) : 0.0f;
    String s = "{";
    s += "\"enabled\":true,";
    s += "\"hz\":" + String(g_live_pref_hz) + ",";
    s += "\"dt_us\":" + String(g_live_dt_us) + ",";
    s += "\"eff_hz\":" + String(effHz, 1) + ",";
    s += "\"fc\":" + String(cutoff, 1) + ",";
    s += "\"ax\":" + String(g_live_acc_mps2[0], 4) + ",";
    s += "\"ay\":" + String(g_live_acc_mps2[1], 4) + ",";
    s += "\"az\":" + String(g_live_acc_mps2[2], 4) + ",";
    s += "\"mag\":" + String(g_live_mag_acc, 4) + ",";
    s += "\"vx_mmps\":" + String(g_live_vel_mmps[0], 3) + ",";
    s += "\"vy_mmps\":" + String(g_live_vel_mmps[1], 3) + ",";
    s += "\"vz_mmps\":" + String(g_live_vel_mmps[2], 3) + ",";
    s += "\"vmag_mmps\":" + String(g_live_mag_vel_mmps, 3) + ",";
    s += "\"dx_mm\":" + String(g_live_disp_mm[0], 4) + ",";
    s += "\"dy_mm\":" + String(g_live_disp_mm[1], 4) + ",";
    s += "\"dz_mm\":" + String(g_live_disp_mm[2], 4) + ",";
    s += "\"dmag_mm\":" + String(g_live_mag_disp_mm, 4);
    s += "}";
    server.send(200, "application/json", s);
    return;
  }

  // ===== Realtime ISO20816 path: per-axis mean-removed integration,
  //       then 3-axis vector magnitude RMS for output. =====
  float lpf_rt[3] = {0, 0, 0};
  bool lpfInitRt = false;
  float velRt[3] = {0, 0, 0};
  float dispRt[3] = {0, 0, 0};
  double ss_a_mag2 = 0;
  double ss_v_mag2 = 0;
  double ss_d_mag2 = 0;

  for (uint16_t i = 0; i < valid; i++)
  {
    const float a[3] = {bx[i] - mx, by[i] - my, bz[i] - mz};
    if (!lpfInitRt)
    {
      lpf_rt[0] = a[0];
      lpf_rt[1] = a[1];
      lpf_rt[2] = a[2];
      lpfInitRt = true;
    }
    else
    {
      lpf_rt[0] += alpha * (a[0] - lpf_rt[0]);
      lpf_rt[1] += alpha * (a[1] - lpf_rt[1]);
      lpf_rt[2] += alpha * (a[2] - lpf_rt[2]);
    }
    velRt[0] += lpf_rt[0] * dt;
    velRt[1] += lpf_rt[1] * dt;
    velRt[2] += lpf_rt[2] * dt;
    dispRt[0] += velRt[0] * dt;
    dispRt[1] += velRt[1] * dt;
    dispRt[2] += velRt[2] * dt;

    ss_a_mag2 += (double)(lpf_rt[0] * lpf_rt[0] + lpf_rt[1] * lpf_rt[1] + lpf_rt[2] * lpf_rt[2]);
    ss_v_mag2 += (double)(velRt[0] * velRt[0] + velRt[1] * velRt[1] + velRt[2] * velRt[2]);
    ss_d_mag2 += (double)(dispRt[0] * dispRt[0] + dispRt[1] * dispRt[1] + dispRt[2] * dispRt[2]);
  }

  free(bx);
  free(by);
  free(bz);

  const float invRt = 1.0f / (float)valid;
  // Quadrature subtraction of noise floor from acc magnitude RMS:
  // rms_signal = sqrt(rms_total^2 - rms_noise^2) when uncorrelated.
  const float rms_a_raw = sqrtf((float)(ss_a_mag2 * invRt));
  const float rms_a = (rms_a_raw > g_rt_noise_mps2)
                          ? sqrtf(rms_a_raw * rms_a_raw - g_rt_noise_mps2 * g_rt_noise_mps2)
                          : 0.0f;
  const float rms_v = sqrtf((float)(ss_v_mag2 * invRt)) * 1000.0f;
  const float rms_d = sqrtf((float)(ss_d_mag2 * invRt)) * 1000.0f;

  g_rt_hist_acc_mps2[g_rt_hist_idx] = rms_a;
  g_rt_hist_vel_mmps[g_rt_hist_idx] = rms_v;
  g_rt_hist_disp_mm[g_rt_hist_idx] = rms_d;
  g_rt_hist_idx = (g_rt_hist_idx + 1) % 10;
  if (g_rt_hist_count < 10)
    g_rt_hist_count++;

  auto avgN = [&](float *arr, uint8_t take) {
    if (!g_rt_hist_count)
      return 0.0f;
    const uint8_t use = (g_rt_hist_count < take) ? g_rt_hist_count : take;
    double sum = 0;
    for (uint8_t i = 0; i < use; i++)
      sum += arr[(uint8_t)((g_rt_hist_idx + 10 - i - 1) % 10)];
    return (float)(sum / (double)use);
  };

  const float avgAcc5 = avgN(g_rt_hist_acc_mps2, 5);
  const float avgVel5 = avgN(g_rt_hist_vel_mmps, 5);
  const float avgDisp5 = avgN(g_rt_hist_disp_mm, 5);
  const float avgAcc10 = avgN(g_rt_hist_acc_mps2, 10);
  const float avgVel10 = avgN(g_rt_hist_vel_mmps, 10);
  const float avgDisp10 = avgN(g_rt_hist_disp_mm, 10);

  const float effHz = (g_live_dt_us > 0) ? (1e6f / (float)g_live_dt_us) : 0.0f;
  String s = "{";
  s += "\"enabled\":true,";
  s += "\"realtime\":true,";
  s += "\"hz\":" + String(g_live_pref_hz) + ",";
  s += "\"dt_us\":" + String(g_live_dt_us) + ",";
  s += "\"eff_hz\":" + String(effHz, 1) + ",";
  s += "\"noise_mps2\":" + String(g_rt_noise_mps2, 4) + ",";
  s += "\"avg1\":{\"acc_mps2\":" + String(rms_a, 3) + ",";
  s += "\"vel_mmps\":" + String(rms_v, 3) + ",";
  s += "\"disp_mm\":" + String(rms_d, 4) + "},";
  s += "\"avg5\":{\"acc_mps2\":" + String(avgAcc5, 3) + ",";
  s += "\"vel_mmps\":" + String(avgVel5, 3) + ",";
  s += "\"disp_mm\":" + String(avgDisp5, 4) + "},";
  s += "\"avg10\":{\"acc_mps2\":" + String(avgAcc10, 3) + ",";
  s += "\"vel_mmps\":" + String(avgVel10, 3) + ",";
  s += "\"disp_mm\":" + String(avgDisp10, 4) + "}";
  s += "}";
  server.send(200, "application/json", s);
}
