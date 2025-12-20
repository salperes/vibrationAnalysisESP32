#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <WiFi.h>

#include "api_handlers.h"
#include "app_state.h"
#include "config.h"

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

  Serial.println("[WiFi] STA connect failed, switching to AP");
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
  if (!ok)
  {
    Serial.println("[WiFi] AP start failed; check power/reset.");
  }
}

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] setup() start");

  if (!LittleFS.begin(true))
  {
    Serial.println("LittleFS mount/format FAIL");
    while (1)
      delay(1000);
  }

  Wire.begin(21, 22);
  Wire.setClock(400000);

  g_i2cMutex = xSemaphoreCreateMutex();
  if (!g_i2cMutex)
  {
    Serial.println("I2C mutex alloc FAIL");
    while (1)
      delay(1000);
  }
  Serial.println("[BOOT] I2C + mutex ready");

  startWiFiOrAP();
  Serial.println("[BOOT] WiFi/AP init done");

  rebuildListCache();
  Serial.println("[BOOT] FS list cache built");
  registerRoutes();
  Serial.println("[BOOT] Routes registered");

  server.begin();
  Serial.println("[BOOT] HTTP server started");

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
