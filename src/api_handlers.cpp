#include <Arduino.h>
#include <Update.h>

#include "api_handlers.h"
#include "config.h"
#include "html_pages.h"
#include "sensor_utils.h"
#include "recording.h"
#include "calibration.h"
#include "analysis.h"
#include "live_preview.h"
#include "file_handlers.h"
#include "sensor_utils.h"
#include "trigger.h"

// ======================= Version =======================
static String versionJson()
{
  String s = "{";
  s += "\"version\":\"" APP_VERSION "\",";
  s += "\"hash\":\"" BUILD_HASH "\",";
  s += "\"built\":\"" __DATE__ " " __TIME__ "\"";
  s += "}";
  return s;
}

// ======================= Info =======================
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
  s += "\"apSsid\":\"" + g_apSsid + "\",";
  s += "\"calibrated\":";
  s += (g_calibPresent ? "true" : "false");
  s += ",";
  s += "\"serial\":\"" + jsonEscape(String(g_device_serial)) + "\",";
  s += "\"deviceName\":\"" + jsonEscape(String(g_device_name)) + "\",";

  // Trigger / grab-mode state
  const char *trigName = "Idle";
  switch (g_trigState) {
    case TrigState::Idle:      trigName = "Idle";      break;
    case TrigState::Armed:     trigName = "Armed";     break;
    case TrigState::Triggered: trigName = "Triggered"; break;
    case TrigState::PostTail:  trigName = "PostTail";  break;
  }
  s += "\"trigState\":\"";
  s += trigName;
  s += "\",";
  s += "\"trigBaseline\":" + String(g_trigBaseline, 4) + ",";
  s += "\"trigCurrentRms\":" + String(g_trigCurrentRms, 4) + ",";
  s += "\"trigThreshold\":" + String(g_trigEffThreshold, 4) + ",";

  uint32_t now = millis();
  uint32_t armedMs = (g_trigState != TrigState::Idle && g_trigArmedAtMs)
                         ? (now - g_trigArmedAtMs) : 0;
  uint32_t firedMs = (g_trigFiredAtMs)
                         ? (now - g_trigFiredAtMs) : 0;
  s += "\"trigArmedMs\":" + String(armedMs) + ",";
  s += "\"trigFiredMs\":" + String(firedMs) + ",";

  // Live preview sensor config (runtime adjustable from the Live tab).
  s += "\"liveHz\":" + String(g_live_pref_hz) + ",";
  s += "\"liveFs\":" + String(g_live_pref_fs_g);

  s += "}";
  return s;
}

// ======================= Device identity =======================
static void handleApiDeviceGet()
{
  String s = "{";
  s += "\"serial\":\"" + jsonEscape(String(g_device_serial)) + "\",";
  s += "\"name\":\"" + jsonEscape(String(g_device_name)) + "\"";
  s += "}";
  server.send(200, "application/json", s);
}

static void handleApiDevicePost()
{
  if (g_recording || g_calibratingStatic || g_calibrating6 || g_trigState != TrigState::Idle)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }
  if (!server.hasArg("name"))
  {
    server.send(400, "text/plain", "Missing name");
    return;
  }
  String name = server.arg("name");
  name.trim();
  if (name.length() == 0 || name.length() >= 24)
  {
    server.send(400, "text/plain", "Name length 1..23");
    return;
  }
  if (!saveDeviceName(name.c_str()))
  {
    server.send(500, "text/plain", "Save failed");
    return;
  }
  server.send(200, "text/plain", "OK");
}

// ======================= Simple handlers =======================
static void handleRoot() { server.send(200, "text/html", ROOT_HTML); }
static void handlePing() { server.send(200, "text/plain", "PONG"); }
static void handleApiInfo() { server.send(200, "application/json", infoJson()); }
static void handleApiVersion() { server.send(200, "application/json", versionJson()); }

static void handleApiReset()
{
  if (g_recording || g_calibratingStatic || g_calibrating6 || g_trigState != TrigState::Idle)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }
  server.send(200, "text/plain", "OK rebooting");
  delay(150);
  ESP.restart();
}

// ======================= OTA firmware update =======================
static void handleUpdateGet()
{
  if (g_recording || g_calibratingStatic || g_calibrating6 || g_trigState != TrigState::Idle)
  {
    server.send(409, "text/plain", "Busy");
    return;
  }
  server.send(200, "text/html", UPDATE_HTML);
}

static void handleUpdatePost()
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

static void handleUpdateUpload()
{
  if (g_recording || g_calibratingStatic || g_calibrating6 || g_trigState != TrigState::Idle)
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

    if (g_recording || g_calibratingStatic || g_calibrating6 || g_trigState != TrigState::Idle)
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

    bool ok = Update.end(true);
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

// ======================= Route registration =======================
void registerRoutes()
{
  server.on("/", handleRoot);
  server.on("/ping", handlePing);

  server.on("/api/info", handleApiInfo);
  server.on("/api/list", handleApiList);
  server.on("/api/fsinfo", handleApiFsInfo);

  server.on("/api/start", HTTP_POST, handleApiStart);
  server.on("/api/stop", HTTP_POST, handleApiStop);

  server.on("/download", handleDownload);
  server.on("/download_csv", handleDownloadCSV);
  server.on("/api/delete", HTTP_POST, handleApiDelete);

  server.on("/api/calibrate_static", HTTP_POST, handleApiCalibrateStatic);
  server.on("/api/calibrate6", HTTP_POST, handleApiCalibrate6);

  server.on("/api/live", handleApiLive);
  server.on("/api/realtime", HTTP_POST, handleApiRealtimeConfig);
  server.on("/api/live_config", HTTP_POST, handleApiLiveConfig);
  server.on("/api/rawmask", HTTP_POST, handleApiRawMask);

  server.on("/api/reset", HTTP_POST, handleApiReset);
  server.on("/update", HTTP_GET, handleUpdateGet);
  server.on("/update", HTTP_POST, handleUpdatePost, handleUpdateUpload);

  server.on("/api/version", handleApiVersion);
  server.on("/api/analyze", handleApiAnalyze);
  server.on("/api/fft", handleApiFFT);

  server.on("/api/device", HTTP_GET, handleApiDeviceGet);
  server.on("/api/device", HTTP_POST, handleApiDevicePost);

  server.on("/api/trigger_arm",    HTTP_POST, handleApiTriggerArm);
  server.on("/api/trigger_disarm", HTTP_POST, handleApiTriggerDisarm);
}
