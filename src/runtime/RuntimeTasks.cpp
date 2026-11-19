#include "runtime/RuntimeTasks.h"

#include <Arduino.h>
#include <Wire.h>
#include <atomic>
#include <cstdint>
#include <climits>

#include "config/RuntimeConfig.h"
#include "runtime/AcquisitionSchedule.h"
#include "runtime/ImuSampleQueue.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu/Lsm6ds3.h"

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

    esp_timer_handle_t acquisitionTimer = nullptr;

    std::atomic<uint32_t> acquisitionHeartbeat{0};
    std::atomic<uint32_t> processingHeartbeat{0};
    std::atomic<uint32_t> telemetryHeartbeat{0};

    std::atomic<int> acquisitionCore{0};
    std::atomic<int> processingCore{0};
    std::atomic<int> telemetryCore{0};

    std::atomic<uint32_t> acquisitionSequence{0};
    std::atomic<uint32_t> missedDeadlineCount{0};
    std::atomic<uint32_t> lastWakeLatenessUs{0};
    std::atomic<uint32_t> maxWakeLatenessUs{0};

    std::atomic<uint32_t> processedSamples{0};
    std::atomic<uint32_t> invalidSamples{0};
    std::atomic<uint32_t> processingSequenceGaps{0};
    std::atomic<uint32_t> lastProcessedSequence{0};

    uint64_t timerEpochUs = 0;

    vbt::imu::Lsm6ds3 imuSensor{};
    std::atomic<uint32_t> successfulImuSamples{0};
    std::atomic<uint32_t> failedImuReads{0};


    // Runs in the ESP timer task context, not as an ISR.
    // Do not perform I/O or sensor transactions here.
    void acquisitionTimerCallback(void*)
    {
        TaskHandle_t handle = acquisitionTaskHandle;

        if(handle == nullptr) return;

        xTaskNotifyGive(handle);
    }

    void updateMaximumLateness(uint32_t value)
    {
        uint32_t previous = maxWakeLatenessUs.load(std::memory_order_relaxed);

        while(value > previous && !maxWakeLatenessUs.compare_exchange_weak(previous, value, std::memory_order_relaxed,std::memory_order_relaxed))
        {   
        }
    }

    void acquisitionTask(void* /*parameter*/)
    {

        acquisitionCore.store(xPortGetCoreID(), std::memory_order_relaxed);

        for(;;)
        {
           // Block until one or more periodic timer notifications arrive.
           const uint32_t notifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

           if(notifications == 0) continue;

           const uint32_t missed = missedNotifications(notifications);

           if(missed > 0) missedDeadlineCount.fetch_add(missed, std::memory_order_relaxed);

           // Advance the sequence for every timer notification received.
           const uint32_t sequence = acquisitionSequence.fetch_add(notifications, std::memory_order_relaxed) + notifications;
           const uint64_t scheduledDeadlineUs = acquisitionDeadlineUs(timerEpochUs, sequence, ACQUISITION_PERIOD_US);
           const uint64_t actualWakeUs = static_cast<uint64_t>(esp_timer_get_time());
           const uint64_t latenessUs = actualWakeUs > scheduledDeadlineUs ? actualWakeUs -scheduledDeadlineUs : 0;
           const uint64_t lateness32 = latenessUs > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(latenessUs);

           lastWakeLatenessUs.store(lateness32, std::memory_order_relaxed);
           updateMaximumLateness(lateness32);
           acquisitionHeartbeat.fetch_add(notifications, std::memory_order_relaxed);

            vbt::RawImuSample sample{};

            const uint64_t sampleTimestampsUs = static_cast<uint64_t>(esp_timer_get_time());

            if(imuSensor.readSample(sample, sampleTimestampsUs, sequence))
            {
                successfulImuSamples.fetch_add(1, std::memory_order_relaxed);
                (void)rawImuQueue.tryPush(sample);
            }
            else
            {
                failedImuReads.fetch_add(1, std::memory_order_relaxed);
            }

        }
    }

    void processingTask(void* /*parameter*/)
    {
        processingCore.store(xPortGetCoreID(), std::memory_order_relaxed);
 
        uint32_t expectedSequence = 0;
        bool havePreviousSequence = false;

        for (;;)
        {
            
            vbt::RawImuSample sample{};

            bool receivedAny = false;

            //Drain all currently available samples
            while(rawImuQueue.tryPop(sample))
            {
                receivedAny = true;
                const bool valid = 
                    vbt::hasStatus(sample.status, vbt::SampleStatus::Valid) &&
                    vbt::hasStatus(sample.status, vbt::SampleStatus::AccelorometerOk) &&
                    vbt::hasStatus(sample.status, vbt::SampleStatus::GyroscopeOk) &&
                    vbt::hasStatus(sample.status, vbt::SampleStatus::TimestampValid) &&
                    vbt::hasStatus(sample.status, vbt::SampleStatus::SequenceValid);

                if(!valid)
                {
                    invalidSamples.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }

                //Detect sequence discontinuities between samples
                if(havePreviousSequence && sample.sequence != expectedSequence)
                {
                    processingSequenceGaps.fetch_add(sample.sequence - expectedSequence, std::memory_order_relaxed);
                }

                expectedSequence = sample.sequence + 1;
                havePreviousSequence = true;

                lastProcessedSequence.store(sample.sequence, std::memory_order_relaxed);
                processedSamples.fetch_add(1, std::memory_order_relaxed);
              
            }

            processingHeartbeat.fetch_add(1, std::memory_order_relaxed);

            if(!receivedAny)
            {
                vTaskDelay(pdMS_TO_TICKS(1));
            }

        }
    }

    #define SC(x) static_cast<unsigned long>(x)
    
    void telemetryTask(void*)
    {
        telemetryCore.store(xPortGetCoreID(), std::memory_order_relaxed);

        for(;;)
        {
            telemetryHeartbeat.fetch_add(1, std::memory_order_relaxed);

            Serial.printf("[RUNTIME] cores acquisition=%d processing=%d telemetry=%d\n",
                acquisitionCore.load(std::memory_order_relaxed),
                processingCore.load(std::memory_order_relaxed),
                telemetryCore.load(std::memory_order_relaxed));
            
            Serial.printf("[ACQ] target=%lu Hz period=%llu us sequence=%lu missed_notifications=%lu last_lateness=%lu us max_lateness=%lu us\n", SC(ACQUISITION_RATE_HZ), static_cast<unsigned long long>(ACQUISITION_PERIOD_US), SC(acquisitionSequence.load(std::memory_order_relaxed)), SC(missedDeadlineCount.load(std::memory_order_relaxed)), SC(lastWakeLatenessUs.load(std::memory_order_relaxed)),SC(maxWakeLatenessUs.load(std::memory_order_relaxed)));
                
            Serial.printf("[HEARTBEAT] acquisition=%lu processing=%lu telemetry=%lu\n", SC(acquisitionHeartbeat.load(std::memory_order_relaxed)), SC(processingHeartbeat.load(std::memory_order_relaxed)), SC(telemetryHeartbeat.load(std::memory_order_relaxed)));

            const UBaseType_t acquisitionStackHwm = acquisitionTaskHandle ? uxTaskGetStackHighWaterMark(acquisitionTaskHandle) : 0;
            const UBaseType_t processingStackHwm = processingTaskHandle ? uxTaskGetStackHighWaterMark(processingTaskHandle) : 0;
            const UBaseType_t telemetryStackHwm = telemetryTaskHandle ? uxTaskGetStackHighWaterMark(telemetryTaskHandle) : 0;

            Serial.printf("[STACK] acquisition_hwm=%lu processing_hwm=%lu telemetry_hwm=%lu\n",SC(acquisitionStackHwm), SC(processingStackHwm), SC(telemetryStackHwm));
            Serial.printf("[QUEUE] raw_imu=%lu/%lu dropped=%lu\n", SC(rawImuQueue.sizeApprox()), SC(rawImuQueue.capacity()), SC(rawImuQueue.droppedCount()));
            Serial.printf("[IMU] samples_ok=%lu read_failures=%lu\n", SC(successfulImuSamples.load(std::memory_order_relaxed)), SC(failedImuReads.load(std::memory_order_relaxed)));
            Serial.printf("[PROCESS] processed=%lu invalid=%lu sequence_gaps=%lu last_sequence=%lu\n", SC(processedSamples.load(std::memory_order_relaxed)), SC(invalidSamples.load(std::memory_order_relaxed)), SC(processingSequenceGaps.load(std::memory_order_relaxed)), SC(lastProcessedSequence.load(std::memory_order_relaxed)));

            vTaskDelay(pdMS_TO_TICKS(1000));
        }

    }
} // namespace

namespace vbt::runtime
{

    bool initializeImu()
    {
        //Initialize Arduino's default 12c pins for this board
        Wire.begin();

        // The sensor and touch controller share this bus.
        // Use 400 kHz for the acquisition workload
        Wire.setClock(400000);

        if(!imuSensor.begin(Wire)) return false;

        return true;
    }

    bool startTasks()
    {
        acquisitionTaskHandle = xTaskCreateStaticPinnedToCore(acquisitionTask, "acquisition", ACQUISITION_STACK_BYTES, nullptr, ACQUISITION_PRIORITY, acquisitionTaskStack, &acquisitionTaskControlBlock, ACQUISITION_CORE);
        if(acquisitionTaskHandle == nullptr) return false;

         processingTaskHandle = xTaskCreateStaticPinnedToCore( processingTask, "processing", PROCESSING_STACK_BYTES, nullptr, PROCESSING_PRIORITY, processingTaskStack, &processingTaskControlBlock, PROCESSING_CORE);
        if (processingTaskHandle == nullptr) return false;


        telemetryTaskHandle = xTaskCreateStaticPinnedToCore(telemetryTask, "telemetry", TELEMETRY_STACK_BYTES, nullptr, TELEMETRY_PRIORITY, telemetryTaskStack, &telemetryTaskControlBlock, TELEMETRY_CORE);
        if (telemetryTaskHandle == nullptr) return false;

        esp_timer_create_args_t timerArgs{};
        timerArgs.callback = acquisitionTimerCallback;
        timerArgs.arg = nullptr;
        timerArgs.dispatch_method = ESP_TIMER_TASK;
        timerArgs.name = "vbt_acquisition";

        if (esp_timer_create(&timerArgs, &acquisitionTimer) != ESP_OK)
        {
            return false;
        }

        timerEpochUs =
            static_cast<uint64_t>(esp_timer_get_time());

        const esp_err_t timerResult = esp_timer_start_periodic(
            acquisitionTimer, ACQUISITION_PERIOD_US);

        if (timerResult != ESP_OK)
        {
            esp_timer_delete(acquisitionTimer);
            acquisitionTimer = nullptr;
            return false;
        }

        return true; 
    }
}
