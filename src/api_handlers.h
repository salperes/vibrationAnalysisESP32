#pragma once

#include "app_state.h"

// Register all HTTP routes on the shared server instance.
void registerRoutes();

// Refresh cached file list (LittleFS) metadata.
void rebuildListCache();

// Configure raw LSB mask bits (0/off, 2,3,4).
void setRawMaskBits(uint8_t bits);
