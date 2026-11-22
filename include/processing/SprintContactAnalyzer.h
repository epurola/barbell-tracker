#pragma once

#include <cstdint>

#include "processing/ActivityAnalyzer.h"

namespace vbt::processing
{
    class SprintContactAnalyzer final : public ActivityAnalyzer
    {
    public:
        enum class Phase : uint8_t
        {
            Idle = 0,
            Stance,
            Aerial
        };

        static constexpr uint32_t MAX_TRACKED_STEPS = 32;
        static constexpr uint32_t MIN_VALID_SPRINT_STEPS = 4;

        static constexpr float TOUCHDOWN_IMPACT_ACCEL_MPS2 = 10.0f;
        static constexpr float PROPULSION_ACCEL_MPS2 = 3.0f;
        static constexpr float TOEOFF_FREEFALL_ACCEL_MPS2 = -4.5f;

        static constexpr uint64_t MIN_CONTACT_TIME_US = 60000;   // 60 ms fastest realistic sprint contact
        static constexpr uint64_t MAX_CONTACT_TIME_US = 260000;  // 260 ms max sprint contact
        static constexpr uint64_t MIN_FLIGHT_TIME_US = 50000;    // 50 ms min aerial time between steps
        static constexpr uint64_t MAX_FLIGHT_TIME_US = 300000;   // 300 ms max aerial time
        static constexpr uint64_t SPRINT_END_TIMEOUT_US = 600000; // 600 ms with no step ends the sprint bout

        ActivityMode mode() const override { return ActivityMode::SprintContact; }
        bool isEffortInProgress() const override { return phase_ != Phase::Idle; }

        Phase phase() const { return phase_; }
        uint32_t currentStepCount() const { return stepCount_; }
        const SprintContactMetrics& lastCompletedSprint() const { return lastCompletedSprint_; }

    protected:
        void onReset() override;
        bool onSample(const KinematicSample& sample, bool stationary, ActivityOutput& output) override;
        bool onFinishRecording(ActivityOutput& output) override;

    private:
        void resetCurrentBout();
        void HandleIdle(float az, uint64_t timestampUs, bool stationary);
        bool HandleStance(float az, uint64_t timestampUs, bool stationary, SprintContactMetrics& output);
        bool HandleAerial(float az, uint64_t timestampUs, bool stationary, SprintContactMetrics& output);
        void recordCompletedContact(float contactTimeMs);
        bool finalizeSprintBout(uint64_t timestampUs, SprintContactMetrics& output);

        Phase phase_{Phase::Idle};
        bool sawPropulsion_{false};

        uint32_t stepCount_{0};
        uint32_t flightCount_{0};

        uint64_t sprintStartTimestampUs_{0};
        uint64_t lastStepEventTimestampUs_{0};
        uint64_t touchdownTimestampUs_{0};
        uint64_t toeoffTimestampUs_{0};

        float lastContactTimeMs_{0.0f};
        float contactTimeSumMs_{0.0f};
        float minContactTimeMs_{0.0f};
        float flightTimeSumMs_{0.0f};

        float oddStepContactSumMs_{0.0f};
        uint32_t oddStepCount_{0};
        float evenStepContactSumMs_{0.0f};
        uint32_t evenStepCount_{0};

        float stepContactTimesMs_[MAX_TRACKED_STEPS]{};

        SprintContactMetrics lastCompletedSprint_{};
    };
}
