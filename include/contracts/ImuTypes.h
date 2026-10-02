#pragma once

#include <cstdint>

#include "common/Status.h"
#include "common/Vector3.h"

namespace
{
    struct RawimuSample
    {
        uint64_t timestamp_us{0};
        uint32_t sequence{0};

        Vector3 accelerometer{};
        Vector3 gyroscope{};

        SampleStatus status{SampleStatus::None};
    };
}