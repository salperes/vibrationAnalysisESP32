#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include "LIS2DW12_ESP32.h"
#include "app_state.h"

// ---- Cached live-preview sensor (shared across modules) ----
extern LIS2DW12 g_liveSensor;
extern bool g_liveSensorReady;

// ---- File validation ----
bool isSafeAccelFile(String p);
bool fileExists(const String &path);

// ---- String helpers ----
String jsonEscape(const String &s);
bool isValidYYMMDDHHMMSS(const String &ts);
String makeNewFileNameFromUI(const String &ts12);

// ---- Header read/write ----
// Reads the file's header (V3 or V4) into a version-agnostic ParsedHeader.
// Caller does NOT need to seek; this function leaves the file positioned
// at the first sample (i.e. at offset out.header_bytes). Returns false on
// I/O error, bad magic, or unsupported version.
bool readParsedHeader(File &f, ParsedHeader &out);
bool rewriteHeaderSamples(const String &path, uint32_t samplesWritten);

// ---- FullScale conversion ----
LIS2DW12::FullScale fsFromG(uint8_t fs_g);
uint8_t fsToByte(LIS2DW12::FullScale fs);

// ---- Pose name ----
const char *poseName(int step);

// ---- Analysis math ----
float mgPerLsb(uint8_t res_bits, uint8_t fs_g);
float rawAlignedToG(int16_t rawAligned, uint8_t res_bits, uint8_t fs_g);
float applyCal1(float g, float offset, float scale);
float rmsFromSumSq(double sumSq, uint32_t n);

// ---- Live preview state ----
void resetLivePreviewState();

// ---- Calibration ----
// Reads NVS calibration blob into g_liveSensor and updates g_calibPresent.
// Safe to call before sensor I2C init (loadCalibrationNVS uses Preferences only).
void loadCalibrationAtBoot();
