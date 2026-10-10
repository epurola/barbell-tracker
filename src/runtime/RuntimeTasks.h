#include "runtime/RuntimeTasks.h"

#include <Arduino.h>

#include "config/RuntimeConfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace 
{
    using namespace vbt::runtime; 

    StaticTask_t acquisitionTaskControlBlock;
    StaticTask_t processingTaskControlBlock;
    StaticTask_t telemetryTaskControlBlock;

    StackType_t acquisitionTaskStack[ACQUISITION_STACK_BYTES / sizeof(StackType_t)];
    StackType_t processingTaskStack[PROCESSING_STACK_BYTES / sizeof(StackType_t)];
    StackType_t telemetryTaskStack[TELEMETRY_STACK_BYTES / sizeof(StackType_t)];

    TaskHandle_t acquisitionTaskHandle = nullptr;
    TaskHandle_t processingTaskHandle  = nullptr;
    TaskHandle_t telemetryTaskHandle   = nullptr;

    volatile uint32_t acquisitionHeartbeat = 0;
    volatile uint32_t processingHeartbeat  = 0;
    volatile uint32_t telemetryHeartbeat   = 0;

    volatile int acquisitionObservedCore = -1;
    volatile int processingObservedCore  = -1;
    volatile int telemetryObservedCore   = -1;

    void acquisitionTask(void* /*parameter*/)
    {
        for(;;)
        {
            ++acquisitionHeartbeat;

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    void processingTask(void* /*parameter*/)
    {
        processingObservedCore = xPortGetCoreID();

        for (;;)
        {
            ++processingHeartbeat;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    void telemetryTask(void*)
    {
        telemetryObservedCore = xPortGetCoreID();

        for(;;)
        {
            ++telemetryHeartbeat;
            Serial.printf(
                "[RUNTIME] acquisition(core=%d hb=%lu) "
                "processing(core%d hb%lu) "
                "telemetry(core=%d hb%lu)\n",
                acquisitionObservedCore,
                static_cast<unsigned long>(acquisitionHeartbeat),
                processingObservedCore,
                static_cast<unsigned long>(processingHeartbeat),
                telemetryObservedCore,
                static_cast<unsigned long>(telemetryHeartbeat));

            vTaskDelay(pdMS_TO_TICKS(1000));
        }

    }
} // namespace

namespace vbt::runtime
{
    bool startTasks()
    {
        acquisitionTaskHandle = xTaskCreateStaticPinnedToCore(
                acquisitionTask,
                "acquisition",
                ACQUISITION_STACK_BYTES,
                nullptr,
                ACQUISITION_PRIORITY,
                acquisitionTaskStack,
                &acquisitionTaskControlBlock,
                ACQUISITION_CORE
        );
        if(acquisitionTaskHandle == nullptr) return false;

         processingTaskHandle = xTaskCreateStaticPinnedToCore(
            processingTask,
            "processing",
            PROCESSING_STACK_BYTES,
            nullptr,
            PROCESSING_PRIORITY,
            processingTaskStack,
            &processingTaskControlBlock,
            PROCESSING_CORE
        );
        if (processingTaskHandle == nullptr) return false;


        telemetryTaskHandle = xTaskCreateStaticPinnedToCore(
            telemetryTask,
            "telemetry",
            TELEMETRY_STACK_BYTES,
            nullptr,
            TELEMETRY_PRIORITY,
            telemetryTaskStack,
            &telemetryTaskControlBlock,
            TELEMETRY_CORE
        );
        if (telemetryTaskHandle == nullptr) return false;


        return true; 
    }
}
