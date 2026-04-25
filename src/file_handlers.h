#pragma once

#include <Arduino.h>

// Cache management
void rebuildListCache();
void setRawMaskBits(uint8_t bits);
// Returns true if LittleFS has at least `bytesNeeded` free (plus an 8 KB
// safety margin) available. Used by /api/start and /api/trigger_arm to
// reject recordings that would overflow the partition.
bool hasFreeSpaceFor(size_t bytesNeeded);

// HTTP handlers
void handleApiList();
void handleApiFsInfo();
void handleDownload();
void handleDownloadCSV();
void handleApiDelete();
void handleApiRawMask();
