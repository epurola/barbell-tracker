#pragma once

#include <cstdint>

namespace vbt
{
    enum class RepPhase : uint8_t
    {
        Idle = 0,
        Eccentric,
        Concentric,
        Lockout
    };

    struct RepMetrics
    {
        static constexpr uint32_t MAX_SET_REPS = 10;

        uint32_t rep_number{0};
        uint64_t start_timestamp_us{0};
        uint64_t concentric_start_us{0};
        uint64_t concentric_end_us{0};
        uint64_t end_timestamp_us{0};

        // Latest rep metrics
        float mean_concentric_velocity_mps{0.0f};
        float peak_concentric_velocity_mps{0.0f};
        float peak_concentric_accel_mps2{0.0f};
        float range_of_motion_m{0.0f};

        // Full set summary (e.g. 6x6 Smith Machine Squat set breakdown)
        float rep_mean_velocity_mps[MAX_SET_REPS]{};
        float rep_peak_velocity_mps[MAX_SET_REPS]{};
        float best_mean_velocity_mps{0.0f};
        float set_mean_velocity_mps{0.0f};
        float velocity_loss_percent{0.0f};

        RepPhase phase{RepPhase::Idle};
        bool valid{false};
    };
}
