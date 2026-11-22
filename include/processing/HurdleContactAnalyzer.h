#pragma once

#include <cstdint>

#include "processing/ActivityAnalyzer.h"

namespace vbt::processing
{
    class HurdleContactAnalyzer final : public ActivityAnalyzer
    {
    public:
        enum class Phase : uint8_t
        {
            Idle = 0,
            ApproachFlight,
            GroundContact,
            ReboundFlight
        };

        static constexpr float GRAVITY_MPS2 = 9.80665f;
        static constexpr float FREEFALL_ACCEL_MPS2 = -7.5f;
        static constexpr float TOUCHDOWN_IMPACT_ACCEL_MPS2 = 15.0f;
        static constexpr float REBOUND_PUSH_ACCEL_MPS2 = 4.0f;

        static constexpr uint64_t MIN_APPROACH_FLIGHT_US = 100000; // 100 ms in air before touchdown
        static constexpr uint64_t MIN_CONTACT_TIME_US = 80000;     // 80 ms min realistic plyo contact
        static constexpr uint64_t MAX_CONTACT_TIME_US = 450000;    // 450 ms max plyo contact (otherwise stopped)
        static constexpr uint64_t MIN_REBOUND_FLIGHT_US = 150000;  // 150 ms min rebound flight
        static constexpr uint64_t MAX_REBOUND_FLIGHT_US = 1100000; // 1.1 s max rebound flight

        ActivityMode mode() const override { return ActivityMode::HurdleContact; }
        bool isEffortInProgress() const override { return phase_ != Phase::Idle; }

        Phase phase() const { return phase_; }
        uint32_t contactCount() const { return contactCount_; }
        float minContactTimeMs() const { return minContactTimeMs_; }
        const HurdleContactMetrics& lastCompletedContact() const { return lastCompletedContact_; }

    protected:
        void onReset() override;
        bool onSample(const KinematicSample& sample, bool stationary, ActivityOutput& output) override;
        bool onFinishRecording(ActivityOutput& output) override;

    private:
        void resetCurrentHop();
        void HandleIdle(float az, uint64_t timestampUs);
        void HandleApproachFlight(float az, uint64_t timestampUs, bool stationary);
        void HandleGroundContact(float az, uint64_t timestampUs, bool stationary);
        bool HandleReboundFlight(float az, uint64_t timestampUs, HurdleContactMetrics& output);

        Phase phase_{Phase::Idle};
        uint32_t contactCount_{0};
        bool sawReboundPush_{false};

        uint64_t approachTakeoffTimestampUs_{0};
        uint64_t touchdownTimestampUs_{0};
        uint64_t toeoffTimestampUs_{0};

        float minContactTimeMs_{0.0f};
        float contactTimeSumMs_{0.0f};
        float rsiSum_{0.0f};
        HurdleContactMetrics lastCompletedContact_{};
    };
}
