#include <Arduino.h>
#include <Wire.h>

#include "calibration.h"
#include "sensor_utils.h"

// ======================= Calibration tasks =======================
static void calibrateStaticTask(void * /*arg*/)
{
  g_liveSensorReady = false;
  g_calibratingStatic = true;

  if (g_i2cMutex)
  {
    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(5000)) != pdTRUE)
    {
      Serial.println("[CAL] I2C mutex timeout");
      g_calibratingStatic = false;
      vTaskDelete(nullptr);
      return;
    }
  }

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
      g_calibPresent = true;
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
  g_liveSensorReady = false;
  g_calibrating6 = true;
  g_calibStep = 0;

  if (g_i2cMutex)
  {
    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(5000)) != pdTRUE)
    {
      Serial.println("[CAL6] I2C mutex timeout");
      g_calibrating6 = false;
      vTaskDelete(nullptr);
      return;
    }
  }

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
  g_calibPresent = true;

  if (g_i2cMutex)
    xSemaphoreGive(g_i2cMutex);

  g_calibStep = -1;
  g_calibrating6 = false;
  resetLivePreviewState();
  vTaskDelete(nullptr);
}

// ======================= HTTP Handlers =======================
void handleApiCalibrateStatic()
{
  if (g_recording || g_calibratingStatic || g_calibrating6 || g_trigState != TrigState::Idle)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }
  g_calibratingStatic = true; // set before task creation to prevent TOCTOU race
  BaseType_t ok = xTaskCreatePinnedToCore(calibrateStaticTask, "calS", 4096, nullptr, 2, nullptr, 1);
  if (ok != pdPASS)
  {
    g_calibratingStatic = false;
    server.send(500, "text/plain", "Task create failed");
    return;
  }
  server.send(200, "text/plain", "Static calibration started");
}

void handleApiCalibrate6()
{
  if (g_recording || g_calibratingStatic || g_calibrating6 || g_trigState != TrigState::Idle)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }
  g_calibrating6 = true; // set before task creation to prevent TOCTOU race
  BaseType_t ok = xTaskCreatePinnedToCore(calibrate6PosTask, "cal6", 6144, nullptr, 2, nullptr, 1);
  if (ok != pdPASS)
  {
    g_calibrating6 = false;
    server.send(500, "text/plain", "Task create failed");
    return;
  }
  server.send(200, "text/plain", "6-pos calibration started");
}
