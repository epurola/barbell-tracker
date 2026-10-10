#pragma once

namespace vbt::runtime
{
    /**
    * Initializes and starts the application FreeRTOS task architecture.
    *
    * Tasks are statically allocated and pinned to their designated cores.
    *
    * Returns true if all application tasks were created successfully.
    */
    bool startTasks();
}