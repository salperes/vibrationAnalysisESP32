#include <Arduino.h>
#include <Wire.h>

#include "live_preview.h"
#include "sensor_utils.h"
#include "file_handlers.h"

// ======================= Noise measurement =======================
static float measureRealtimeNoise(LIS2DW12 &lis, const uint8_t resBits, const uint8_t fs_g)
{
  const uint16_t samples = 400; // ~0.5s @800 Hz
  double sumSq = 0;
  uint16_t valid = 0;

  auto cal = lis.getCalibration();

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

    float ax = gx * GRAVITY_MPS2;
    float ay = gy * GRAVITY_MPS2;
    float az = gz * GRAVITY_MPS2;

    float mag = sqrtf(ax * ax + ay * ay + az * az);
    float diff = mag - GRAVITY_MPS2;
    sumSq += (double)diff * (double)diff;
    valid++;
    delayMicroseconds(1200);
  }

  if (!valid)
    return 0.0f;
  return sqrtf((float)(sumSq / (double)valid));
}

// ======================= Realtime config =======================
void handleApiRealtimeConfig()
{
  if (g_recording || g_calibratingStatic || g_calibrating6)
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

  Wire.setClock(1000000);
  LIS2DW12 lis(Wire, 0x18);
  if (!lis.begin(-1, -1, 1000000))
  {
    if (g_i2cMutex)
      xSemaphoreGive(g_i2cMutex);
    server.send(500, "text/plain", "Sensor init failed");
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

  g_rt_noise_mps2 = measureRealtimeNoise(lis, lis.activeResolutionBits(), fsToByte(cfg.fs));

  if (g_i2cMutex)
    xSemaphoreGive(g_i2cMutex);

  server.send(200, "text/plain", "Realtime enabled");
}

// ======================= Live preview =======================
void handleApiLive()
{
  if (g_recording || g_calibratingStatic || g_calibrating6)
  {
    server.send(200, "application/json", "{\"enabled\":false}");
    return;
  }

  uint32_t now = millis();
  if (!g_realtimeEnabled && now - g_liveLastMs < 1000)
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

  if (!g_liveSensorReady || g_calDirty)
  {
    if (!g_liveSensor.begin(-1, -1, 1000000))
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
    g_liveSensor.applyConfig(cfg);
    g_liveSensor.setRateHz(LIVE_PREVIEW_HZ);
    g_liveSensor.loadCalibrationNVS("lis2dw12", "cal");
    g_calDirty = false;
    g_liveSensorReady = true;
  }

  auto cal = g_liveSensor.getCalibration();
  const uint8_t resBits = g_liveSensor.activeResolutionBits();
  const uint8_t fs_g = fsToByte(LIS2DW12::FullScale::G2);

  float cutoff = g_live_lp_cut_hz;
  if (server.hasArg("fc"))
  {
    float fcReq = server.arg("fc").toFloat();
    if (fcReq >= 5.0f && fcReq <= (float)LIVE_PREVIEW_HZ * 0.5f)
      cutoff = fcReq;
  }
  g_live_lp_cut_hz = cutoff;

  const uint16_t samples = min((uint16_t)200, LIVE_PREVIEW_HZ);
  const float dt = 1.0f / (float)LIVE_PREVIEW_HZ;
  const float tau = 1.0f / (2.0f * PI * cutoff);
  const float alpha = dt / (tau + dt);

  double sumAcc[3] = {0, 0, 0};
  float vel[3] = {0, 0, 0};
  float disp[3] = {0, 0, 0};
  float lpf[3] = {0, 0, 0};
  bool lpfInit = false;
  uint16_t valid = 0;

  if (!g_realtimeEnabled)
  {
    for (uint16_t i = 0; i < samples; i++)
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
      delayMicroseconds(1200);
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
    return;
  }

  // ===== Realtime ISO20816 path (magnitude-only) =====
  double sumAccMag = 0;
  double sumVelMag = 0;
  double sumDispMag = 0;
  float velMag = 0;
  float dispMag = 0;
  uint16_t validRt = 0;

  for (uint16_t i = 0; i < samples; i++)
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

    float ax = gx * GRAVITY_MPS2;
    float ay = gy * GRAVITY_MPS2;
    float az = gz * GRAVITY_MPS2;

    float accMag = fabsf(sqrtf(ax * ax + ay * ay + az * az) - GRAVITY_MPS2);
    accMag = max(0.0f, accMag - g_rt_noise_mps2);

    velMag += accMag * dt;
    dispMag += velMag * dt;

    sumAccMag += accMag;
    sumVelMag += fabsf(velMag);
    sumDispMag += fabsf(dispMag);
    validRt++;
    delayMicroseconds(1200);
  }

  if (g_i2cMutex)
    xSemaphoreGive(g_i2cMutex);

  if (!validRt)
  {
    server.send(200, "application/json", "{\"enabled\":false}");
    return;
  }

  const float invRt = 1.0f / (float)validRt;
  float avgAcc1s = (float)(sumAccMag * invRt);
  float avgVel1s = (float)(sumVelMag * invRt) * 1000.0f;
  float avgDisp1s = (float)(sumDispMag * invRt) * 1000.0f;

  g_rt_hist_acc_mps2[g_rt_hist_idx] = avgAcc1s;
  g_rt_hist_vel_mmps[g_rt_hist_idx] = avgVel1s;
  g_rt_hist_disp_mm[g_rt_hist_idx] = avgDisp1s;
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

  float avgAcc5 = avgN(g_rt_hist_acc_mps2, 5);
  float avgVel5 = avgN(g_rt_hist_vel_mmps, 5);
  float avgDisp5 = avgN(g_rt_hist_disp_mm, 5);
  float avgAcc10 = avgN(g_rt_hist_acc_mps2, 10);
  float avgVel10 = avgN(g_rt_hist_vel_mmps, 10);
  float avgDisp10 = avgN(g_rt_hist_disp_mm, 10);

  String s = "{";
  s += "\"enabled\":true,";
  s += "\"realtime\":true,";
  s += "\"hz\":" + String(LIVE_PREVIEW_HZ) + ",";
  s += "\"noise_mps2\":" + String(g_rt_noise_mps2, 4) + ",";
  s += "\"avg1\":{\"acc_mps2\":" + String(avgAcc1s, 3) + ",";
  s += "\"vel_mmps\":" + String(avgVel1s, 2) + ",";
  s += "\"disp_mm\":" + String(avgDisp1s, 3) + "},";
  s += "\"avg5\":{\"acc_mps2\":" + String(avgAcc5, 3) + ",";
  s += "\"vel_mmps\":" + String(avgVel5, 2) + ",";
  s += "\"disp_mm\":" + String(avgDisp5, 3) + "},";
  s += "\"avg10\":{\"acc_mps2\":" + String(avgAcc10, 3) + ",";
  s += "\"vel_mmps\":" + String(avgVel10, 2) + ",";
  s += "\"disp_mm\":" + String(avgDisp10, 3) + "}";
  s += "}";
  server.send(200, "application/json", s);
}
