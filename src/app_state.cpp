#include "app_state.h"

WebServer server(80);

String g_updateLastError = "";
size_t g_updateExpected = 0;

SemaphoreHandle_t g_i2cMutex = nullptr;

volatile bool g_recording = false;
volatile bool g_stopRequested = false;

volatile bool g_calibratingStatic = false;
volatile bool g_calibrating6 = false;
volatile int g_calibStep = -1; // 0..5

RecConfig g_cfg{};
RecMetadata g_recMeta{};
volatile uint32_t g_samplesWritten = 0;
volatile uint32_t g_maxBacklog = 0;
volatile uint32_t g_elapsedMs = 0;

TaskHandle_t g_recTask = nullptr;
String g_currentFile = ""; // "/accelYYMMDDHHMMSS.dat"
String g_uiTimestamp = ""; // "YYMMDDHHMMSS"

float g_calibAvg[6][3] = {0};

uint32_t g_liveLastMs = 0;
float g_live_g[3] = {0, 0, 0};
uint16_t g_live_pref_hz   = 800;
uint8_t  g_live_pref_fs_g = 2;
float g_live_lp_cut_hz = 200.0f; // default low-pass cutoff for preview
const float GRAVITY_MPS2 = 9.80665f;
float g_live_acc_mps2[3] = {0, 0, 0};
float g_live_vel_mmps[3] = {0, 0, 0};
float g_live_disp_mm[3] = {0, 0, 0};
float g_live_mag_acc = 0;
float g_live_mag_vel_mmps = 0;
float g_live_mag_disp_mm = 0;
uint32_t g_live_dt_us = 0;
volatile bool g_calDirty = true; // calibration changed -> reload NVS

bool g_apMode = false;
String g_apSsid = "";

char g_device_serial[16] = {0};
char g_device_name[24]   = {0};

String g_listCache = "[]";
uint32_t g_listCacheMs = 0;
const uint32_t LIST_CACHE_TTL_MS = 5000;

bool g_realtimeEnabled = false;
float g_rt_noise_mps2 = 0;
float g_rt_hist_acc_mps2[10] = {0};
float g_rt_hist_vel_mmps[10] = {0};
float g_rt_hist_disp_mm[10] = {0};
uint8_t g_rt_hist_count = 0;
uint8_t g_rt_hist_idx = 0;
uint8_t g_rawMaskBits = 0; // default: no LSB masking (user opts in via UI)

volatile TrigState g_trigState = TrigState::Idle;
volatile bool g_trigDisarmRequested = false;
volatile uint32_t g_trigArmedAtMs = 0;
volatile uint32_t g_trigFiredAtMs = 0;
volatile uint32_t g_trigPostStartMs = 0;

uint16_t g_trigPreS = 3;
uint16_t g_trigPostS = 5;
uint16_t g_trigMaxS = 60;
uint8_t  g_trigMode = 0;
uint8_t  g_trigMult = 3;
float    g_trigManualThr = 0.5f;

volatile float g_trigBaseline = 0.0f;
volatile float g_trigCurrentRms = 0.0f;
volatile float g_trigEffThreshold = 0.0f;

TaskHandle_t g_trigTask = nullptr;

volatile bool g_calibPresent = false;
