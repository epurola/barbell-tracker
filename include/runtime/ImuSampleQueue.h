#pragma once

#include "contracts/ImuTypes.h"
#include "runtime/SpscRingbuffer.h"

#include <cstddef>

namespace vbt::runtime
{
    // 256 slots provide approximately 512 ms of buffering at 500 Hz.
    // Capacity must remain a power of two for SpscRingBuffer
    inline constexpr std::size_t RAW_IMU_QUEUE_CAPACITY = 256;

    inline SpscRingBuffer<RawImuSample, RAW_IMU_QUEUE_CAPACITY> rawImuQueue{};
}