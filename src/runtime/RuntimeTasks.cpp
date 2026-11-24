#include "runtime/RuntimeTasks.h"

#include <Arduino.h>
#include <Wire.h>
#include <atomic>
#include <cstdint>

#include "common/DebugLog.h"
#include "config/RuntimeConfig.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu/Lsm6ds3.h"
#include "processing/ImuPipeline.h"
#include "runtime/AcquisitionSchedule.h"
#include "runtime/GateReceiver.h"
#include "runtime/ImuSampleQueue.h"
#include "runtime/TraceLogger.h"
#include "ui/UiManager.h"
#include "ui/HeadlessWebUi.h"

namespace
{
    using namespace vbt::runtime;

    enum class PendingCommand : uint8_t
    {
        None = 0,
        StartCalibratedSession,
        StartUncalibratedSession,
        StartNextSet,
        StopSession,
        ReturnToModeSelect
    };

    StaticTask_t acquisitionTaskControlBlock;
    StaticTask_t processingTaskControlBlock;
#if VBT_DEBUG_SERIAL
    StaticTask_t telemetryTaskControlBlock;
    StackType_t telemetryTaskStack[TELEMETRY_STACK_BYTES / sizeof(StackType_t)];
    TaskHandle_t telemetryTaskHandle   = nullptr;
    std::atomic<uint32_t> telemetryHeartbeat{0};
    std::atomic<int> telemetryCore{0};
#endif
    StaticTask_t uiTaskControlBlock;

    StackType_t acquisitionTaskStack[ACQUISITION_STACK_BYTES / sizeof(StackType_t)];
    StackType_t processingTaskStack[PROCESSING_STACK_BYTES / sizeof(StackType_t)];
    StackType_t uiTaskStack[UI_STACK_BYTES / sizeof(StackType_t)];

    TaskHandle_t acquisitionTaskHandle = nullptr;
    TaskHandle_t processingTaskHandle  = nullptr;
    TaskHandle_t uiTaskHandle          = nullptr;

    esp_timer_handle_t acquisitionTimer = nullptr;

    std::atomic<uint32_t> acquisitionHeartbeat{0};
    std::atomic<uint32_t> processingHeartbeat{0};
    std::atomic<uint32_t> uiHeartbeat{0};

    std::atomic<int> acquisitionCore{0};
    std::atomic<int> processingCore{0};
    std::atomic<int> uiCore{0};

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
    std::atomic<uint32_t> calibrationSamples{0};
    std::atomic<uint32_t> calibratedSamples{0};
    std::atomic<uint32_t> calibrationFailures{0};
    std::atomic<uint32_t> filteredSamples{0};
    std::atomic<uint32_t> filterFailures{0};
    std::atomic<uint32_t> orientationSamples{0};
    std::atomic<uint32_t> orientationFailures{0};
    std::atomic<uint32_t> kinematicSamples{0};
    std::atomic<uint32_t> kinematicFailures{0};
    std::atomic<uint32_t> completedEfforts{0};

    std::atomic<uint8_t> pendingCommand{static_cast<uint8_t>(PendingCommand::None)};
    std::atomic<uint8_t> requestedMode{static_cast<uint8_t>(vbt::ActivityMode::BarbellVbt)};
    std::atomic<uint8_t> currentMode{static_cast<uint8_t>(vbt::ActivityMode::BarbellVbt)};
    std::atomic<uint8_t> currentSessionState{static_cast<uint8_t>(vbt::SessionState::IdleModeSelect)};
    std::atomic<bool> effortInProgress{false};

    vbt::ui::UiManager uiManager{};

    portMUX_TYPE resultSnapshotMux = portMUX_INITIALIZER_UNLOCKED;
    vbt::ActivityOutput latestResultSnapshot{};

    void publishActivityResult(const vbt::ActivityOutput& activity)
    {
        portENTER_CRITICAL(&resultSnapshotMux);
        latestResultSnapshot = activity;
        portEXIT_CRITICAL(&resultSnapshotMux);
    }

    // Runs in the ESP timer task context, not as an ISR.
    // Do not perform I/O or sensor transactions here.
    void acquisitionTimerCallback(void*)
    {
        TaskHandle_t handle = acquisitionTaskHandle;

        if (handle == nullptr) return;

        xTaskNotifyGive(handle);
    }

    void updateMaximumLateness(uint32_t value)
    {
        uint32_t previous = maxWakeLatenessUs.load(std::memory_order_relaxed);

        while (value > previous && !maxWakeLatenessUs.compare_exchange_weak(previous, value, std::memory_order_relaxed, std::memory_order_relaxed)){}
    }

    void publishPipelineCounters(const vbt::processing::PipelineCounters& counters)
    {
        processedSamples.store(counters.processedSamples, std::memory_order_relaxed);
        invalidSamples.store(counters.invalidSamples, std::memory_order_relaxed);
        processingSequenceGaps.store(counters.sequenceGaps, std::memory_order_relaxed);
        lastProcessedSequence.store(counters.lastSequence, std::memory_order_relaxed);
        calibrationSamples.store(counters.calibrationSamples, std::memory_order_relaxed);
        calibratedSamples.store(counters.calibratedSamples, std::memory_order_relaxed);
        calibrationFailures.store(counters.calibrationFailures, std::memory_order_relaxed);
        filteredSamples.store(counters.filteredSamples, std::memory_order_relaxed);
        filterFailures.store(counters.filterFailures, std::memory_order_relaxed);
        orientationSamples.store(counters.orientationSamples, std::memory_order_relaxed);
        orientationFailures.store(counters.orientationFailures, std::memory_order_relaxed);
        kinematicSamples.store(counters.kinematicSamples, std::memory_order_relaxed);
        kinematicFailures.store(counters.kinematicFailures, std::memory_order_relaxed);
        completedEfforts.store(counters.completedEfforts, std::memory_order_relaxed);
    }

    void handleSessionCommands(vbt::processing::ImuPipeline& pipeline)
    {
        const auto cmd = static_cast<PendingCommand>(pendingCommand.exchange(static_cast<uint8_t>(PendingCommand::None), std::memory_order_acq_rel));

        if (cmd == PendingCommand::None) return;

        const auto mode = static_cast<vbt::ActivityMode>(requestedMode.load(std::memory_order_relaxed));

        auto usesGates = [](vbt::ActivityMode m) {
            return m == vbt::ActivityMode::SprintContact || m == vbt::ActivityMode::BlockStart;
        };

        switch (cmd)
        {
            case PendingCommand::StartCalibratedSession:
                pipeline.startSession(mode, true);
                traceLogger().reset(pipeline.mode());
                if (usesGates(pipeline.mode())) { gateReceiver().begin(); gateReceiver().reset(); }
                else { gateReceiver().end(); }
                break;
            case PendingCommand::StartUncalibratedSession:
                pipeline.startSession(mode, false);
                traceLogger().reset(pipeline.mode());
                if (usesGates(pipeline.mode())) { gateReceiver().begin(); gateReceiver().reset(); }
                else { gateReceiver().end(); }
                break;
            case PendingCommand::StartNextSet:
                pipeline.startNextSet();
                traceLogger().reset(pipeline.mode());
                if (usesGates(pipeline.mode())) { gateReceiver().begin(); gateReceiver().reset(); }
                else { gateReceiver().end(); }
                break;
            case PendingCommand::StopSession:
            {
                traceLogger().freeze();
                vbt::ActivityOutput finalOutput{};
                pipeline.stopSession(finalOutput);
                finalOutput.gates = gateReceiver().snapshot();
                gateReceiver().end();
                publishActivityResult(finalOutput);
                break;
            }
            case PendingCommand::ReturnToModeSelect:
                traceLogger().freeze();
                gateReceiver().end();
                pipeline.returnToModeSelect();
                break;
            default: break;
        }

        currentMode.store(static_cast<uint8_t>(pipeline.mode()), std::memory_order_relaxed);
        currentSessionState.store(static_cast<uint8_t>(pipeline.sessionState()), std::memory_order_relaxed);
        effortInProgress.store(pipeline.isEffortInProgress(), std::memory_order_relaxed);
    }

    // Pinned to Core 1 (ACQUISITION_CORE, Priority 5).
    // Dedicated real-time 500 Hz sensor polling task.
    void acquisitionTask(void* /*parameter*/)
    {
        acquisitionCore.store(xPortGetCoreID(), std::memory_order_relaxed);

        for (;;)
        {
            // Block until one or more periodic timer notifications arrive.
            const uint32_t notifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            if (notifications == 0) continue;

            const uint32_t missed = missedNotifications(notifications);

            if (missed > 0) missedDeadlineCount.fetch_add(missed, std::memory_order_relaxed);

            // Advance the sequence for every timer notification received.
            const uint32_t sequence = acquisitionSequence.fetch_add(notifications, std::memory_order_relaxed) + notifications;
            const uint64_t scheduledDeadlineUs = acquisitionDeadlineUs(timerEpochUs, sequence, ACQUISITION_PERIOD_US);
            const uint64_t actualWakeUs = static_cast<uint64_t>(esp_timer_get_time());
            const uint64_t latenessUs = actualWakeUs > scheduledDeadlineUs ? actualWakeUs - scheduledDeadlineUs : 0;
            const uint32_t lateness32 = latenessUs > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(latenessUs);

            lastWakeLatenessUs.store(lateness32, std::memory_order_relaxed);
            updateMaximumLateness(lateness32);
            acquisitionHeartbeat.fetch_add(notifications, std::memory_order_relaxed);

            vbt::RawImuSample sample{};

            const uint64_t sampleTimestampsUs = static_cast<uint64_t>(esp_timer_get_time());

            const auto state = static_cast<vbt::SessionState>(currentSessionState.load(std::memory_order_relaxed));

            if (state == vbt::SessionState::IdleModeSelect || state == vbt::SessionState::ResultsReady) continue;

            if (imuSensor.readSample(sample, sampleTimestampsUs, sequence))
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

    // Pinned to Core 0 (PROCESSING_CORE, Priority 4).
    // Drains raw samples from the SPSC queue and runs the IMU processing pipeline.
    void processingTask(void* /*parameter*/)
    {
        processingCore.store(xPortGetCoreID(), std::memory_order_relaxed);

        static vbt::processing::ImuPipeline pipeline{};
#if VBT_DEBUG_SERIAL
        auto previousCalState = pipeline.calibrationState();
        uint32_t previousCalFailures = 0;
#endif

        for (;;)
        {
            handleSessionCommands(pipeline);

            vbt::RawImuSample sample{};
            bool receivedAny = false;

            // Drain all currently available samples
            while (rawImuQueue.tryPop(sample))
            {
                receivedAny = true;

                vbt::KinematicSample kinematics{};
                vbt::ActivityOutput activity{};
                if (pipeline.process(sample, kinematics, activity))
                {
                    const bool stationary = vbt::hasStatus(kinematics.status, vbt::SampleStatus::Stationary);
                    traceLogger().record(kinematics, pipeline.activePhaseCode(), stationary, activity.completed);
                    effortInProgress.store(activity.effort_in_progress, std::memory_order_relaxed);
                    if (activity.completed) publishActivityResult(activity);
                }

                currentSessionState.store( static_cast<uint8_t>(pipeline.sessionState()), std::memory_order_relaxed);

#if VBT_DEBUG_SERIAL
                const auto& counts = pipeline.counters();
                if (counts.calibrationFailures != previousCalFailures)
                {
                    previousCalFailures = counts.calibrationFailures;
                    VBT_LOGLN("[CALIBRATION] Failed: sensor was not stationary (retrying...)");
                }

                const auto currentCalState = pipeline.calibrationState();
                if (currentCalState != previousCalState)
                {
                    if (currentCalState == vbt::processing::ImuCalibrator::State::Ready)
                    {
                        VBT_LOGLN("[CALIBRATION] Complete — Recording active");
                    }
                    previousCalState = currentCalState;
                }
#endif
            }

            if (receivedAny) publishPipelineCounters(pipeline.counters());

            processingHeartbeat.fetch_add(1, std::memory_order_relaxed);

            if (!receivedAny) vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    // Pinned to Core 0 (UI_CORE, Priority 2).
    // Initializes the SPI display on Core 0 and handles touch/button UI updates.
    void uiTask(void* /*parameter*/)
    {
        uiCore.store(xPortGetCoreID(), std::memory_order_relaxed);

        if (!uiManager.begin())
        {
            VBT_LOGLN("ERROR: UI initialization failed");
            vTaskSuspend(nullptr);
        }

        for (;;)
        {
            uiHeartbeat.fetch_add(1, std::memory_order_relaxed);
            uiManager.update();
            vTaskDelay(pdMS_TO_TICKS(25));
        }
    }

#if VBT_DEBUG_SERIAL
    #define SC(x) static_cast<unsigned long>(x)

    // Pinned to Core 0 (TELEMETRY_CORE, Priority 2).
    // Prints 1 Hz diagnostic telemetry snapshots over Serial.
    void telemetryTask(void*)
    {
        telemetryCore.store(xPortGetCoreID(), std::memory_order_relaxed);

        for (;;)
        {
            telemetryHeartbeat.fetch_add(1, std::memory_order_relaxed);

            Serial.printf("[RUNTIME] cores acquisition=%d processing=%d telemetry=%d ui=%d\n", acquisitionCore.load(std::memory_order_relaxed), processingCore.load(std::memory_order_relaxed), telemetryCore.load(std::memory_order_relaxed), uiCore.load(std::memory_order_relaxed));
            Serial.printf("[ACQ] target=%lu Hz period=%llu us sequence=%lu missed_notifications=%lu last_lateness=%lu us max_lateness=%lu us\n", SC(ACQUISITION_RATE_HZ), static_cast<unsigned long long>(ACQUISITION_PERIOD_US), SC(acquisitionSequence.load(std::memory_order_relaxed)), SC(missedDeadlineCount.load(std::memory_order_relaxed)), SC(lastWakeLatenessUs.load(std::memory_order_relaxed)), SC(maxWakeLatenessUs.load(std::memory_order_relaxed)));
            Serial.printf("[HEARTBEAT] acquisition=%lu processing=%lu telemetry=%lu ui=%lu\n", SC(acquisitionHeartbeat.load(std::memory_order_relaxed)), SC(processingHeartbeat.load(std::memory_order_relaxed)), SC(telemetryHeartbeat.load(std::memory_order_relaxed)), SC(uiHeartbeat.load(std::memory_order_relaxed)));

            const UBaseType_t acquisitionStackHwm = acquisitionTaskHandle ? uxTaskGetStackHighWaterMark(acquisitionTaskHandle) : 0;
            const UBaseType_t processingStackHwm = processingTaskHandle ? uxTaskGetStackHighWaterMark(processingTaskHandle) : 0;
            const UBaseType_t telemetryStackHwm = telemetryTaskHandle ? uxTaskGetStackHighWaterMark(telemetryTaskHandle) : 0;
            const UBaseType_t uiStackHwm = uiTaskHandle ? uxTaskGetStackHighWaterMark(uiTaskHandle) : 0;

            Serial.printf("[STACK] acquisition_hwm=%lu processing_hwm=%lu telemetry_hwm=%lu ui_hwm=%lu\n", SC(acquisitionStackHwm),SC(processingStackHwm),SC(telemetryStackHwm),SC(uiStackHwm));
            Serial.printf("[QUEUE] raw_imu=%lu/%lu dropped=%lu\n", SC(rawImuQueue.sizeApprox()), SC(rawImuQueue.capacity()), SC(rawImuQueue.droppedCount()));
            Serial.printf("[IMU] samples_ok=%lu read_failures=%lu\n", SC(successfulImuSamples.load(std::memory_order_relaxed)), SC(failedImuReads.load(std::memory_order_relaxed)));
            Serial.printf("[PROCESS] processed=%lu invalid=%lu sequence_gaps=%lu last_sequence=%lu\n", SC(processedSamples.load(std::memory_order_relaxed)), SC(invalidSamples.load(std::memory_order_relaxed)), SC(processingSequenceGaps.load(std::memory_order_relaxed)), SC(lastProcessedSequence.load(std::memory_order_relaxed)));
            Serial.printf("[CALIBRATION] collected=%lu calibrated=%lu failures=%lu\n", SC(calibrationSamples.load(std::memory_order_relaxed)), SC(calibratedSamples.load(std::memory_order_relaxed)), SC(calibrationFailures.load(std::memory_order_relaxed)));
            Serial.printf("[FILTER] filtered=%lu failures=%lu\n", SC(filteredSamples.load(std::memory_order_relaxed)), SC(filterFailures.load(std::memory_order_relaxed)));
            Serial.printf("[ORIENTATION] valid=%lu failures=%lu\n", SC(orientationSamples.load(std::memory_order_relaxed)), SC(orientationFailures.load(std::memory_order_relaxed)));
            Serial.printf("[KINEMATICS] valid=%lu failures=%lu\n", SC(kinematicSamples.load(std::memory_order_relaxed)), SC(kinematicFailures.load(std::memory_order_relaxed)));
            Serial.printf("[SESSION] mode=%u state=%u efforts=%lu\n", static_cast<unsigned>(currentMode.load(std::memory_order_relaxed)), static_cast<unsigned>(currentSessionState.load(std::memory_order_relaxed)), SC(completedEfforts.load(std::memory_order_relaxed)));

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
#endif
} // namespace

namespace vbt::runtime
{
    bool initializeImu()
    {
        // Initialize Arduino's default I2C pins for this board
        Wire.begin();

        // The sensor and touch controller share this bus.
        // Use 400 kHz for the acquisition workload
        Wire.setClock(400000);

        if (!imuSensor.begin(Wire)) return false;

        return true;
    }

    bool startTasks()
    {
        acquisitionTaskHandle = xTaskCreateStaticPinnedToCore(acquisitionTask, "acquisition", ACQUISITION_STACK_BYTES, nullptr, ACQUISITION_PRIORITY, acquisitionTaskStack, &acquisitionTaskControlBlock, ACQUISITION_CORE);
        if (acquisitionTaskHandle == nullptr) return false;

        processingTaskHandle = xTaskCreateStaticPinnedToCore(processingTask, "processing", PROCESSING_STACK_BYTES, nullptr, PROCESSING_PRIORITY, processingTaskStack, &processingTaskControlBlock, PROCESSING_CORE);
        if (processingTaskHandle == nullptr) return false;

#if VBT_DEBUG_SERIAL
        telemetryTaskHandle = xTaskCreateStaticPinnedToCore(telemetryTask, "telemetry", TELEMETRY_STACK_BYTES, nullptr, TELEMETRY_PRIORITY, telemetryTaskStack, &telemetryTaskControlBlock, TELEMETRY_CORE);
        if (telemetryTaskHandle == nullptr) return false;
#endif

        uiTaskHandle = xTaskCreateStaticPinnedToCore(uiTask, "ui", UI_STACK_BYTES, nullptr, UI_PRIORITY, uiTaskStack, &uiTaskControlBlock, UI_CORE);
        if (uiTaskHandle == nullptr) return false;

        esp_timer_create_args_t timerArgs{};
        timerArgs.callback = acquisitionTimerCallback;
        timerArgs.arg = nullptr;
        timerArgs.dispatch_method = ESP_TIMER_TASK;
        timerArgs.name = "vbt_acquisition";

        if (esp_timer_create(&timerArgs, &acquisitionTimer) != ESP_OK) return false;

        timerEpochUs = static_cast<uint64_t>(esp_timer_get_time());

        const esp_err_t timerResult = esp_timer_start_periodic(acquisitionTimer, ACQUISITION_PERIOD_US);

        if (timerResult != ESP_OK)
        {
            esp_timer_delete(acquisitionTimer);
            acquisitionTimer = nullptr;
            return false;
        }

        return true;
    }

    void requestStartSession(vbt::ActivityMode mode, bool recalibrate)
    {
        requestedMode.store(static_cast<uint8_t>(mode), std::memory_order_relaxed);
        pendingCommand.store(static_cast<uint8_t>(recalibrate ? PendingCommand::StartCalibratedSession : PendingCommand::StartUncalibratedSession), std::memory_order_release);
    }

    void requestNextSet()
    {
        pendingCommand.store(static_cast<uint8_t>(PendingCommand::StartNextSet), std::memory_order_release);
    }

    void requestStopSession()
    {
        pendingCommand.store(static_cast<uint8_t>(PendingCommand::StopSession), std::memory_order_release);
    }

    void requestModeSelect()
    {
        pendingCommand.store(static_cast<uint8_t>(PendingCommand::ReturnToModeSelect), std::memory_order_release);
    }

    vbt::ActivityMode activeActivityMode()
    {
        return static_cast<vbt::ActivityMode>(currentMode.load(std::memory_order_relaxed));
    }

    vbt::SessionState activeSessionState()
    {
        return static_cast<vbt::SessionState>(currentSessionState.load(std::memory_order_relaxed));
    }

    bool isEffortInProgress()
    {
        return effortInProgress.load(std::memory_order_relaxed);
    }

    vbt::ActivityOutput latestActivityResult()
    {
        vbt::ActivityOutput copy{};
        portENTER_CRITICAL(&resultSnapshotMux);
        copy = latestResultSnapshot;
        portEXIT_CRITICAL(&resultSnapshotMux);
        return copy;
    }
}
