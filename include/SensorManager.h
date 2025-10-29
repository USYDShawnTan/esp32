#pragma once

#include <stddef.h>

class TelemetryClient;

/**
 * Starts the background task that manages motion-driven LED animations.
 * The TelemetryClient pointer is kept for backwards compatibility with
 * previous revisions and may be null.
 */
void startSensorManagerTask(TelemetryClient *client);

// Optional LED overrides for speech events; safe to call from other tasks.
void sensorManagerTriggerSpeechListening();
void sensorManagerTriggerSpeechSpeaking();
void sensorManagerClearSpeechOverride();

// Unlocks monitoring mode (exits blue calibration animation) once startup audio completes.
void sensorManagerUnlockMonitoring();
