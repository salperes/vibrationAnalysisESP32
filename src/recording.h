#pragma once

#include <Arduino.h>

// Hardware timer control
bool startTimerHz(uint16_t hz);
void stopTimer();

// HTTP handlers
void handleApiStart();
void handleApiStop();
