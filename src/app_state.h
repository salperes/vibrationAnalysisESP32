#pragma once

#include <Arduino.h>
#include <WebServer.h>

#include "LIS2DW12_ESP32.h"

#pragma pack(push, 1)
// V3 (legacy, read-only). New recordings always write V4. Kept here so
// previously-recorded files still parse and analyze correctly.
struct FileHeaderV3
{
  char magic[8];     // "LIS2DW12"
  uint16_t version;  // 3
  uint16_t rate_hz;
  uint16_t record_s;
  uint32_t samples;
  uint8_t fs_g;
  uint8_t res_bits;
  uint8_t q_bits;
  uint8_t reserved0;
  float cal_offset_g[3];
  float cal_scale[3];
}; // 48 bytes

// V4 adds trigger info + measurement metadata. New recordings (manual or
// triggered) write this format. Magic is identical to V3 so the same
// "LIS2DW12" prefix gates both; readers branch on the version field.
struct FileHeaderV4
{
  // Identity (10 B)
  char     magic[8];          // "LIS2DW12"
  uint16_t version;           // 4

  // Acquisition (12 B)
  uint16_t rate_hz;
  uint16_t record_s;          // configured cap (s)
  uint32_t samples;           // actual sample count
  uint8_t  fs_g;              // 2/4/8/16
  uint8_t  res_bits;          // 12/14
  uint8_t  q_bits;            // 0/10/12/14
  uint8_t  flags;             // bit0: triggered, bit1: truncated

  // Calibration (24 B)
  float cal_offset_g[3];
  float cal_scale[3];

  // Trigger info (12 B)  -- zero-filled for manual recordings
  uint32_t pre_samples;       // count of pre-roll samples in stream
  float    threshold_used;    // m/s^2 RMS threshold that fired
  uint8_t  trig_mode;         // 0=auto-baseline, 1=manual, 0xFF=N/A
  uint8_t  trig_mult;         // multiplier (auto mode); 0 otherwise
  uint16_t reserved;

  // Metadata (156 B, null-padded ASCII)
  char serial_no[16];
  char device_name[24];
  char meas_point[24];
  char scan_dir[12];          // "RADIAL_H","RADIAL_V","AXIAL"
  char operator_name[16];
  char notes[64];
}; // 214 bytes

struct Sample6
{
  int16_t ax, ay, az; // aligned raw
};
#pragma pack(pop)

static_assert(sizeof(FileHeaderV3) == 46, "FileHeaderV3 size mismatch");
static_assert(sizeof(FileHeaderV4) == 214, "FileHeaderV4 size mismatch");

// Version-agnostic header view used by analyze / FFT / CSV. V3 fields are
// always present; V4-only fields are zero-filled when reading a V3 file.
struct ParsedHeader
{
  uint16_t version;        // 3 or 4
  uint32_t header_bytes;   // size to skip past to reach the first sample

  uint16_t rate_hz;
  uint16_t record_s;
  uint32_t samples;
  uint8_t  fs_g;
  uint8_t  res_bits;
  uint8_t  q_bits;
  uint8_t  flags;          // V4 only; 0 for V3

  float    cal_offset_g[3];
  float    cal_scale[3];

  // V4 trigger info (zeroed for V3)
  uint32_t pre_samples;
  float    threshold_used;
  uint8_t  trig_mode;
  uint8_t  trig_mult;

  // V4 metadata (empty strings for V3)
  char serial_no[16];
  char device_name[24];
  char meas_point[24];
  char scan_dir[12];
  char operator_name[16];
  char notes[64];
};

struct RecConfig
{
  uint16_t hz = 100; // UI: 2 -> 1.6Hz (LP special), 13 -> 12.5, etc.
  uint16_t sec = 60;
  uint8_t fs_g = 2;
  uint8_t qBits = 0;
  LIS2DW12::Mode mode = LIS2DW12::Mode::HighPerf; // LP/HP
};

// Per-recording user-supplied metadata. Filled by /api/start (manual) or
// /api/trigger_arm (grab) before the recording task starts. Stored verbatim
// in FileHeaderV4 so each .dat is self-describing.
struct RecMetadata
{
  char meas_point[24];
  char scan_dir[12];
  char operator_name[16];
  char notes[64];
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
extern RecMetadata g_recMeta;
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
extern uint32_t g_live_dt_us; // measured per-sample period during last live read
extern volatile bool g_calDirty; // calibration changed -> reload NVS

extern bool g_apMode;
extern String g_apSsid;

// Device identity (populated at boot from MAC + NVS). Embedded in every
// FileHeaderV4 so each .dat file is traceable to its source device.
extern char g_device_serial[16]; // hex of MAC's last 6 bytes; immutable
extern char g_device_name[24];   // user-set, persisted in NVS

extern String g_listCache;
extern uint32_t g_listCacheMs;
extern const uint32_t LIST_CACHE_TTL_MS;

// Realtime ISO20816 mode state
extern bool g_realtimeEnabled;
extern float g_rt_noise_mps2;
extern float g_rt_hist_acc_mps2[10];
extern float g_rt_hist_vel_mmps[10];
extern float g_rt_hist_disp_mm[10];
extern uint8_t g_rt_hist_count;
extern uint8_t g_rt_hist_idx;

// Bitmasking of raw sensor LSBs: 0=off, 2,3,4 bits
extern uint8_t g_rawMaskBits;

// True when a valid calibration blob was found in NVS (set at boot and after
// successful save). Read by /api/info to inform the UI.
extern volatile bool g_calibPresent;
