#pragma once

#include <cstdint>

#include "common/Status.h"
#include "common/Vector3.h"


namespace vbt
{
    /**
    * Estimated physical state in the world frame.
    *
    * Units:
    *   acceleration: m/s²
    *   velocity:     m/s
    *   position:     m
    */

    struct KinematicSample
    {
        uint64_t timestamp_us{0};
        uint32_t sequence{0};

        Vector3 linear_acceleration_mps2{};
        Vector3 velocity_mps{};
        Vector3 position_m{};

        SampleStatus status{SampleStatus::None};
    };
}