#pragma once

// HTTP handlers for grab-mode (trigger-armed) recording.
//   POST /api/trigger_arm    -> spawn triggerTask, enter ARMED
//   POST /api/trigger_disarm -> request disarm; task cleans up
void handleApiTriggerArm();
void handleApiTriggerDisarm();
