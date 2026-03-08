#pragma once

#include <Arduino.h>

// Cache management
void rebuildListCache();
void setRawMaskBits(uint8_t bits);

// HTTP handlers
void handleApiList();
void handleApiFsInfo();
void handleDownload();
void handleDownloadCSV();
void handleApiDelete();
