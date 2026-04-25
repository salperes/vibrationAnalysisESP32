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

// V4 (legacy, read-only). New recordings write V5. Kept here so any V4 .dat
// files captured during early grab-mode testing still parse and analyze.
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

// V5 extends V4 with two more metadata strings: the serial number of the
// machine being scanned (user-supplied) and the firmware version that
// produced the file (auto from APP_VERSION). Layout up through `notes` is
// identical to V4; new fields are appended.
struct FileHeaderV5
{
  // Identity (10 B) -- magic and version match V4's prefix
  char     magic[8];          // "LIS2DW12"
  uint16_t version;           // 5

  // Acquisition (12 B)
  uint16_t rate_hz;
  uint16_t record_s;
  uint32_t samples;
  uint8_t  fs_g;
  uint8_t  res_bits;
  uint8_t  q_bits;
  uint8_t  flags;             // bit0: triggered, bit1: truncated

  // Calibration (24 B)
  float cal_offset_g[3];
  float cal_scale[3];

  // Trigger info (12 B)
  uint32_t pre_samples;
  float    threshold_used;
  uint8_t  trig_mode;
  uint8_t  trig_mult;
  uint16_t reserved;

  // Per-recording metadata strings (156 B)
  char serial_no[16];         // device's own serial (auto from MAC)
  char device_name[24];
  char meas_point[24];
  char scan_dir[12];
  char operator_name[16];
  char notes[64];

  // V5 additions (36 B)
  char scanned_system_serial[24]; // serial of the machine being scanned
  char firmware_version[12];      // e.g. "V3.9.10"
}; // 250 bytes

struct Sample6
{
  int16_t ax, ay, az; // aligned raw
};
#pragma pack(pop)

static_assert(sizeof(FileHeaderV3) == 46,  "FileHeaderV3 size mismatch");
static_assert(sizeof(FileHeaderV4) == 214, "FileHeaderV4 size mismatch");
static_assert(sizeof(FileHeaderV5) == 250, "FileHeaderV5 size mismatch");

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

  // V5 metadata (empty strings for V3/V4)
  char scanned_system_serial[24];
  char firmware_version[12];
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
// in FileHeaderV5 so each .dat is self-describing. Device's own serial and
// firmware version come from globals/APP_VERSION (not RecMetadata).
struct RecMetadata
{
  char meas_point[24];
  char scan_dir[12];
  char operator_name[16];
  char notes[64];
  char scanned_system_serial[24]; // serial of the machine being measured
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
// Live preview sensor configuration (runtime adjustable via /api/live_config).
// Setter must also set g_calDirty=true so the next /api/live re-applies them.
extern uint16_t g_live_pref_hz;   // sample rate Hz (100/200/400/800/1600)
extern uint8_t  g_live_pref_fs_g; // full-scale range (2/4/8/16)
extern float g_live_lp_cut_hz; // default low-pass cutoff for preview

// ----- Live noise floor (per-axis + magnitude RMS captured by /api/live_zero)
// When g_live_zero_active is true, handleApiLive applies quadrature
// subtraction to the freshly-computed RMS values: out = sqrt(max(0, raw^2 - noise^2)).
// Persisted in NVS under namespace "livezero".
extern volatile bool g_live_zero_active;
extern float g_live_noise_acc_mps2[3];
extern float g_live_noise_vel_mmps[3];
extern float g_live_noise_disp_mm[3];
extern float g_live_noise_mag_acc;
extern float g_live_noise_mag_vel_mmps;
extern float g_live_noise_mag_disp_mm;
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

// ----- Trigger ("grab") mode state -----
enum class TrigState : uint8_t { Idle = 0, Armed = 1, Triggered = 2, PostTail = 3 };

extern volatile TrigState g_trigState;
extern volatile bool g_trigDisarmRequested;
extern volatile uint32_t g_trigArmedAtMs;     // millis when ARMED was entered
extern volatile uint32_t g_trigFiredAtMs;     // millis when TRIGGERED entered (0 if not)
extern volatile uint32_t g_trigPostStartMs;   // millis when POST_TAIL entered

// User-supplied configuration (latched on /api/trigger_arm)
extern uint16_t g_trigPreS;
extern uint16_t g_trigPostS;
extern uint16_t g_trigMaxS;
extern uint8_t  g_trigMode;                   // 0=auto-baseline, 1=manual
extern uint8_t  g_trigMult;                   // multiplier for auto mode
extern float    g_trigManualThr;              // m/s^2 (manual mode)

// Live status (updated by trigger task; consumed by /api/info)
extern volatile float g_trigBaseline;         // measured baseline RMS (auto mode)
extern volatile float g_trigCurrentRms;       // last evaluated 100 ms window
extern volatile float g_trigEffThreshold;     // computed threshold actually used

extern TaskHandle_t g_trigTask;

// True when a valid calibration blob was found in NVS (set at boot and after
// successful save). Read by /api/info to inform the UI.
extern volatile bool g_calibPresent;
