#pragma once

#include <cstdint>

#include "common/Status.h"
#include "common/Vector3.h"

namespace vbt
{
    struct RawImuSample
    {
        uint64_t timestamp_us{0};
        uint32_t sequence{0};

        vbt::Vector3 accelerometer{};
        vbt::Vector3 gyroscope{};

        SampleStatus status{SampleStatus::None};
    };

    struct CalibratedImuSample
    {
        uint64_t timestamp_us{0};
        uint32_t sequence{0};

        vbt::Vector3 accelerometer_mps2{};
        vbt::Vector3 gyroscope_radps{};

        SampleStatus status{SampleStatus::None};
    };

    struct FilteredImuSample
    {
        uint64_t timestamp_us{0};
        uint32_t sequence{0};

        vbt::Vector3 accelerometer_mps2{};
        vbt::Vector3 gyroscope_radps{};

        SampleStatus status{SampleStatus::None};
    };
}