#pragma once

#include <cstdint>

#include "processing/ActivityAnalyzer.h"

namespace vbt::processing
{
    class BarbellRepAnalyzer final : public ActivityAnalyzer
    {
    public:
        static constexpr float ECCENTRIC_START_VELOCITY_MPS = -0.10f;
        static constexpr float CONCENTRIC_START_VELOCITY_MPS = 0.08f;
        static constexpr float CONCENTRIC_END_VELOCITY_MPS = 0.05f;
        static constexpr float MIN_RANGE_OF_MOTION_M = 0.15f;
        static constexpr uint64_t MIN_CONCENTRIC_DURATION_US = 150000; // 150 ms
        static constexpr uint64_t MAX_PHASE_DURATION_US = 8000000;     // 8 s timeout

        ActivityMode mode() const override { return ActivityMode::BarbellVbt; }
        bool isEffortInProgress() const override { return phase_ != RepPhase::Idle; }

        RepPhase phase() const { return phase_; }
        uint32_t repCount() const { return repCount_; }
        const RepMetrics& lastCompletedRep() const { return lastCompletedRep_; }

    protected:
        void onReset() override;
        bool onSample(const KinematicSample& sample, bool stationary, ActivityOutput& output) override;
        bool onFinishRecording(ActivityOutput& output) override;

    private:
        void resetCurrentRep();
        void startConcentric(uint64_t timestampUs, float positionZ);
        bool finalizeRep(uint64_t timestampUs, RepMetrics& output);

        void HandleIdle(float vz, float pz, const KinematicSample& sample);
        void HandleEccentric(float pz, float az, float vz, const KinematicSample& sample, bool stationary);
        bool HandleConcentric(float pz, float az, float vz, const KinematicSample& sample, bool stationary, RepMetrics& output);

        RepPhase phase_{RepPhase::Idle};
        uint32_t repCount_{0};
        float setMeanVelocitySum_{0.0f};
        RepMetrics lastCompletedRep_{};

        uint64_t repStartTimestampUs_{0};
        uint64_t concentricStartTimestampUs_{0};
        float minPositionZ_{0.0f};
        float maxPositionZ_{0.0f};
        float concentricVelocitySum_{0.0f};
        uint32_t concentricSampleCount_{0};
        float peakVelocityMps_{0.0f};
        float peakAccelMps2_{0.0f};
    };
}
