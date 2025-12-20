#pragma once

// Build info can be overridden via build flags:
// -D APP_VERSION="\"V3.1\"" -D BUILD_HASH="\"a1b2c3d\""
#ifndef APP_VERSION
#define APP_VERSION "V3.6"
#endif

#ifndef BUILD_HASH
#define BUILD_HASH "nogit"
#endif

extern const char *WIFI_SSID;
extern const char *WIFI_PASS;
