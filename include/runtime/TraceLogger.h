#pragma once

#include <cstddef>
#include <cstdint>

#include "contracts/FirmwareContracts.h"
#include "contracts/ModeTypes.h"

namespace vbt::runtime
{
    struct TraceSample
    {
        uint32_t dt_ms{0};
        int16_t az_cmps2{0}; // vertical linear accel Z * 100 (m/s^2)
        int16_t vz_mmps{0};  // vertical velocity Z * 1000 (m/s)
        int16_t pz_mm{0};    // vertical position Z * 1000 (mm)
        uint8_t phase{0};    // active analyzer state-machine phase enum
        uint8_t flags{0};    // bit 0: stationary, bit 1: completed event
    };

    // Single shared static ring buffer (~30 KB) capturing the most recent ~10 seconds
    // at 250 Hz (plus exact 500 Hz state-machine transition ticks).
    class TraceLogger
    {
    public:
        static constexpr size_t CAPACITY = 2500;
        static constexpr uint8_t DECIMATION = 2; // 500 Hz -> 250 Hz (4 ms)

        static constexpr uint8_t FLAG_STATIONARY = 1u << 0;
        static constexpr uint8_t FLAG_COMPLETED  = 1u << 1;

        void reset(vbt::ActivityMode mode);
        void record(const vbt::KinematicSample& sample, uint8_t phase, bool stationary, bool completed);
        void freeze();

        vbt::ActivityMode mode() const { return mode_; }
        size_t count() const { return count_; }
        bool isFrozen() const { return frozen_; }

        // Reads sample at chronological index [0 .. count()-1].
        bool sampleAt(size_t logicalIndex, TraceSample& out) const;

    private:
        static int16_t clampInt16(float scaled);

        TraceSample buffer_[CAPACITY]{};
        size_t writeIndex_{0};
        size_t count_{0};
        uint64_t startTimestampUs_{0};
        uint8_t decimCounter_{0};
        uint8_t lastPhase_{0};
        vbt::ActivityMode mode_{vbt::ActivityMode::BarbellVbt};
        volatile bool frozen_{true};
    };

    TraceLogger& traceLogger();
}
