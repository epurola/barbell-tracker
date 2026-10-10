#pragma once 

#include <cstdint>

namespace vbt
{
    enum class RepPhase :uint8_t
    {
        Idle = 0,
        Eccentric,
        Concentric,
        Lockout
    };

    struct RepMetrics
    {
        uint32_t rep_number{0};

        uint64_t start_timestamp_us{0};
        uint64_t concentric_start_us{0};
        uint64_t consentric_end_us{0};
        uint64_t end_timestamp_us{0};

        float mean_concentric_velocity_mps{0.0f};
        float peak_concentric_velocity_mps{0.0f};

        RepPhase phase{RepPhase::Idle};

        bool valid{false};
    };
}