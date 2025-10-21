#pragma once

#include "TelemetryClient.h"

/**
 * Starts the background task that handles fall detection, vital signs,
 * temperature monitoring, LED status rendering, telemetry uploads, and
 * alert audio queueing. The TelemetryClient instance must remain valid
 * for the lifetime of the firmware.
 */
void startSensorManagerTask(TelemetryClient *client);

