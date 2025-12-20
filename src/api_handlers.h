#pragma once

#include "app_state.h"

// Register all HTTP routes on the shared server instance.
void registerRoutes();

// Refresh cached file list (LittleFS) metadata.
void rebuildListCache();
