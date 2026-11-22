#pragma once

#include <cstdint>

#include "processing/ActivityAnalyzer.h"

namespace vbt::processing
{
    class StandingJumpAnalyzer final : public ActivityAnalyzer
    {
    public:
        enum class Phase : uint8_t
        {
            Idle = 0,
            Propulsion,
            Flight
        };

        static constexpr float GRAVITY_MPS2 = 9.80665f;
        static constexpr float DIP_START_VELOCITY_MPS = -0.15f;
        static constexpr float PUSH_START_ACCEL_MPS2 = 3.0f;
        static constexpr float MIN_TAKEOFF_VELOCITY_MPS = 1.0f;
        static constexpr float TAKEOFF_FREEFALL_ACCEL_MPS2 = -7.5f;
        static constexpr float LANDING_IMPACT_ACCEL_MPS2 = 12.0f;
        static constexpr uint64_t MIN_FLIGHT_TIME_US = 200000;          // 200 ms (~4.9 cm)
        static constexpr uint64_t MAX_FLIGHT_TIME_US = 1200000;         // 1.2 s safety limit
        static constexpr uint64_t MAX_PROPULSION_DURATION_US = 2500000; // 2.5 s timeout

        ActivityMode mode() const override { return ActivityMode::StandingJump; }
        bool isEffortInProgress() const override { return phase_ != Phase::Idle; }

        Phase phase() const { return phase_; }
        uint32_t jumpCount() const { return jumpCount_; }
        const JumpMetrics& lastCompletedJump() const { return lastCompletedJump_; }

    protected:
        void onReset() override;
        bool onSample(const KinematicSample& sample, bool stationary, ActivityOutput& output) override;
        bool onFinishRecording(ActivityOutput& output) override;

    private:
        void resetCurrentJump();
        void HandleIdle(float vz, float az, uint64_t timestampUs, bool stationary);
        void HandlePropulsion(float vz, float az, uint64_t timestampUs, bool stationary);
        bool HandleFlight(float az, uint64_t timestampUs, JumpMetrics& output);

        Phase phase_{Phase::Idle};
        bool armed_{false};
        uint32_t jumpCount_{0};
        float jumpHeightSumCm_{0.0f};
        uint64_t propulsionStartTimestampUs_{0};
        uint64_t takeoffTimestampUs_{0};
        float peakVelocityMps_{0.0f};
        float takeoffVelocityMps_{0.0f};
        JumpMetrics lastCompletedJump_{};
    };
}
