#pragma once

#include <Arduino.h>
#include <WebServer.h>

#include "LIS2DW12_ESP32.h"

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

struct RecConfig
{
  uint16_t hz = 100; // UI: 2 -> 1.6Hz (LP special), 13 -> 12.5, etc.
  uint16_t sec = 60;
  uint8_t fs_g = 2;
  uint8_t qBits = 0;
  LIS2DW12::Mode mode = LIS2DW12::Mode::HighPerf; // LP/HP
};

extern WebServer server;

extern String g_updateLastError;
extern size_t g_updateExpected;

extern SemaphoreHandle_t g_i2cMutex;

extern volatile bool g_recording;
extern volatile bool g_stopRequested;

extern volatile bool g_calibratingStatic;
extern volatile bool g_calibrating6;
extern volatile int g_calibStep; // 0..5

extern RecConfig g_cfg;
extern volatile uint32_t g_samplesWritten;
extern volatile uint32_t g_maxBacklog;
extern volatile uint32_t g_elapsedMs;

extern TaskHandle_t g_recTask;
extern String g_currentFile; // "/accelYYMMDDHHMMSS.dat"
extern String g_uiTimestamp; // "YYMMDDHHMMSS"

extern float g_calibAvg[6][3];

extern uint32_t g_liveLastMs;
extern float g_live_g[3];
extern const uint16_t LIVE_PREVIEW_HZ;
extern float g_live_lp_cut_hz; // default low-pass cutoff for preview
extern const float GRAVITY_MPS2;
extern float g_live_acc_mps2[3];
extern float g_live_vel_mmps[3];
extern float g_live_disp_mm[3];
extern float g_live_mag_acc;
extern float g_live_mag_vel_mmps;
extern float g_live_mag_disp_mm;
extern volatile bool g_calDirty; // calibration changed -> reload NVS

extern bool g_apMode;
extern String g_apSsid;

extern String g_listCache;
extern uint32_t g_listCacheMs;
extern const uint32_t LIST_CACHE_TTL_MS;
