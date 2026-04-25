#pragma once

#include <Arduino.h>

// ROOT_HTML serves "/" and contains a tabbed UI: Live (default), Grab,
// Preview, Update. UPDATE_HTML remains reachable at "/update" as a
// standalone fallback for direct firmware uploads.
extern const char ROOT_HTML[]   PROGMEM;
extern const char UPDATE_HTML[] PROGMEM;
