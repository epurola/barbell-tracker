#pragma once

#include <cstdint>

#include "processing/ActivityAnalyzer.h"

namespace vbt::processing
{
    class BlockStartAnalyzer final : public ActivityAnalyzer
    {
    public:
        enum class Phase : uint8_t
        {
            Idle = 0,
            SetPosition,
            FirstStepsStance,
            FirstStepsAerial
        };

        static constexpr uint32_t TARGET_BLOCK_STEPS = 4;

        static constexpr float BLOCK_CLEARANCE_ACCEL_MPS2 = 4.0f;
        static constexpr float STEP_TOUCHDOWN_ACCEL_MPS2 = 8.0f;
        static constexpr float STEP_TOEOFF_ACCEL_MPS2 = -2.5f;

        static constexpr uint64_t SET_HOLD_DURATION_US = 300000;  // 300 ms stationary in blocks to lock set position
        static constexpr uint64_t MIN_STEP_CONTACT_US = 70000;    // 70 ms min contact time
        static constexpr uint64_t MAX_BLOCK_PUSH_US = 650000;     // 650 ms max initial push from dead stop
        static constexpr uint64_t MAX_STEP_CONTACT_US = 350000;   // 350 ms max follow-up step contact
        static constexpr uint64_t MIN_STEP_AERIAL_US = 40000;     // 40 ms min aerial phase
        static constexpr uint64_t MAX_STEP_AERIAL_US = 300000;    // 300 ms max aerial phase

        ActivityMode mode() const override { return ActivityMode::BlockStart; }
        bool isEffortInProgress() const override
        {
            return phase_ == Phase::FirstStepsStance || phase_ == Phase::FirstStepsAerial;
        }
        uint8_t phaseCode() const override { return static_cast<uint8_t>(phase_); }

        Phase phase() const { return phase_; }
        uint32_t capturedSteps() const { return stepCount_; }
        const BlockStartMetrics& lastCompletedStart() const { return lastCompletedStart_; }

    protected:
        void onReset() override;
        bool onSample(const KinematicSample& sample, bool stationary, ActivityOutput& output) override;
        bool onFinishRecording(ActivityOutput& output) override;

    private:
        void resetCurrentStart();
        void HandleIdle(uint64_t timestampUs, bool stationary);
        void HandleSetPosition(const KinematicSample& sample, bool stationary);
        bool HandleFirstStepsStance(const KinematicSample& sample, bool stationary, BlockStartMetrics& output);
        bool HandleFirstStepsAerial(const KinematicSample& sample, bool stationary, BlockStartMetrics& output);
        bool finalizeBlockStart(BlockStartMetrics& output);

        static float horizontalAccelerationMps2(const Vector3& linearAccel);
        static float estimateDriveAngleDeg(const Vector3& linearAccel);

        Phase phase_{Phase::Idle};
        uint32_t stepCount_{0};

        uint64_t stationaryStartTimestampUs_{0};
        uint64_t startTimestampUs_{0};
        uint64_t stepTouchdownTimestampUs_{0};
        uint64_t stepToeoffTimestampUs_{0};

        float setPitchDeg_{0.0f};
        float currentStepPeakAngleDeg_{0.0f};
        float peakDriveAccelMps2_{0.0f};

        float stepContactTimesMs_[TARGET_BLOCK_STEPS]{};
        float stepDriveAnglesDeg_[TARGET_BLOCK_STEPS]{};

        BlockStartMetrics lastCompletedStart_{};
    };
}
