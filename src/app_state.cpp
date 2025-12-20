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
volatile uint32_t g_samplesWritten = 0;
volatile uint32_t g_maxBacklog = 0;
volatile uint32_t g_elapsedMs = 0;

TaskHandle_t g_recTask = nullptr;
String g_currentFile = ""; // "/accelYYMMDDHHMMSS.dat"
String g_uiTimestamp = ""; // "YYMMDDHHMMSS"

float g_calibAvg[6][3] = {0};

uint32_t g_liveLastMs = 0;
float g_live_g[3] = {0, 0, 0};
const uint16_t LIVE_PREVIEW_HZ = 800;
float g_live_lp_cut_hz = 200.0f; // default low-pass cutoff for preview
const float GRAVITY_MPS2 = 9.80665f;
float g_live_acc_mps2[3] = {0, 0, 0};
float g_live_vel_mmps[3] = {0, 0, 0};
float g_live_disp_mm[3] = {0, 0, 0};
float g_live_mag_acc = 0;
float g_live_mag_vel_mmps = 0;
float g_live_mag_disp_mm = 0;
volatile bool g_calDirty = true; // calibration changed -> reload NVS

bool g_apMode = false;
String g_apSsid = "";

String g_listCache = "[]";
uint32_t g_listCacheMs = 0;
const uint32_t LIST_CACHE_TTL_MS = 2000;
