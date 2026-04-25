#pragma once

// HTTP handlers
void handleApiLive();
void handleApiRealtimeConfig();
// POST /api/live_config  hz=<allowed> [fs=<2|4|8|16>]
// Updates g_live_pref_hz / g_live_pref_fs_g and forces the cached live
// sensor to reinit on the next /api/live call. Rejected while busy.
void handleApiLiveConfig();

// POST /api/live_zero
// Runs a fresh ~250 ms measurement, captures per-axis + magnitude RMS
// values into g_live_noise_*, persists to NVS, and activates quadrature
// subtraction in subsequent /api/live calls. Rejected if the device is
// busy or realtime mode is enabled.
void handleApiLiveZero();
// POST /api/live_zero_clear
// Clears the noise floor (in-RAM + NVS) and disables zero subtraction.
void handleApiLiveZeroClear();
