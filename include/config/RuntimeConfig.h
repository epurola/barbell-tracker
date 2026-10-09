#pragma once 

#include <stdint.h>

namespace vbt::runtime
{
    //Core affinity
    constexpr uint8_t ACQUISITION_CORE = 1;
    constexpr uint8_t PROCESSING_CORE = 0;
    constexpr uint8_t TELEMETRY_CORE = 0;

    // Task priorities
    constexpr uint8_t ACQUISITION_PRIORITY = 5;
    constexpr uint8_t PROCESSING_PRIORITY = 4;
    constexpr uint8_t TELEMETRY_PRIORITY = 2;

    //Static task stack sizes
    constexpr uint32_t ACQUISITION_STACK_BYTES = 4096;
    constexpr uint32_t PROCESSING_STACK_BYTES = 4096;
    constexpr uint32_t TELEMETRY_STACK_BYTES= 3072;

}