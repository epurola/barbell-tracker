#pragma once

#include <cstdint>

#include "common/Status.h"

namespace vbt
{
    struct Quaternion
    {
        float w{1.0f};
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};
    };

    /**
    * Orientation estimate produced by the AHRS.
    *
    * Quaternion convention:
    *   Represents rotation from body frame to world frame.
    */
    struct OrientationState
    {
        uint64_t timestamp_us{0};
        uint32_t sequence{0};

        Quaternion body_to_world{};

        SampleStatus status{SampleStatus::None};
    };
}