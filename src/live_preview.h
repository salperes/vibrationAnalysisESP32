#pragma once

// HTTP handlers
void handleApiLive();
void handleApiRealtimeConfig();
// POST /api/live_config  hz=<allowed> [fs=<2|4|8|16>]
// Updates g_live_pref_hz / g_live_pref_fs_g and forces the cached live
// sensor to reinit on the next /api/live call. Rejected while busy.
void handleApiLiveConfig();
