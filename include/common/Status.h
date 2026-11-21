#pragma once

#include <cstdint>

namespace vbt
{
    enum class SampleStatus : uint32_t
    {
        None             = 0,
        Valid            = 1u << 0,
        AccelorometerOk  = 1u << 1,
        GyroscopeOk      = 1u << 2,
        TimestampValid   = 1u << 3, 
        SequenceValid    = 1u << 4,
        Calibrated       = 1u << 5,
        Filtered         = 1u << 6,
        OrientationValid = 1u << 7,
        KinematicsValid  = 1u << 8
    };

    constexpr SampleStatus operator | (SampleStatus lhs, SampleStatus rhs)
    {
        return static_cast<SampleStatus>(static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs));
    }

    constexpr uint32_t toBits(SampleStatus status)
    {
        return static_cast<uint32_t>(status);
    }

    constexpr bool hasStatus(SampleStatus value, SampleStatus flag)
    {
        return (toBits(value) & toBits(flag)) != 0;
    }
}
