#pragma once

namespace vbt::runtime
{
    // Initialize the shared I2C bus and verify/configure the IMU.
    // Must succeed before starting the runtime tasks.
    bool initializeImu();

    // Initializes and starts the application FreeRTOS task architecture.
    bool startTasks();
}
