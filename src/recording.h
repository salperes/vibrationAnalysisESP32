#pragma once

#include <Arduino.h>

// Hardware timer control (shared across recording + trigger modes; they are
// mutually exclusive via the global busy check).
bool startTimerHz(uint16_t hz);
void stopTimer();

// Atomically read-and-clear the timer's "due sample" counter. Returns the
// number of timer ticks that fired since the last call.
uint32_t consumeTimerDue();
// Force-reset the counter (useful at task startup before sampling begins).
void resetTimerDue();

// HTTP handlers
void handleApiStart();
void handleApiStop();
