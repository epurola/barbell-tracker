#pragma once

#include "contracts/ModeTypes.h"

namespace vbt::runtime
{
    // Initialize the shared I2C bus and verify/configure the IMU.
    // Must succeed before starting the runtime tasks.
    bool initializeImu();

    // Initializes and starts the application FreeRTOS task architecture.
    bool startTasks();

    // UI / Session Control Commands (processed safely by processingTask):
    // Start a session in `mode` (recalibrates for 1s when `recalibrate == true`).
    void requestStartSession(vbt::ActivityMode mode, bool recalibrate = true);

    // Start the next set in the current mode without recalibrating gyro bias.
    void requestNextSet();

    // Stop the active recording window and finalize the set/run results for display.
    void requestStopSession();

    // Return to the swipeable mode selection carousel (`SessionState::IdleModeSelect`).
    void requestModeSelect();

    // State queries for the UI task:
    vbt::ActivityMode activeActivityMode();
    vbt::SessionState activeSessionState();
    bool isEffortInProgress();

    // Thread-safe copy of the latest completed set/activity result.
    vbt::ActivityOutput latestActivityResult();
}
