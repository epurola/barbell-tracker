#include "runtime/TraceLogger.h"

#include <cmath>

namespace vbt::runtime
{
    int16_t TraceLogger::clampInt16(float scaled)
    {
        if (!std::isfinite(scaled)) return 0;
        if (scaled > 32767.0f) return 32767;
        if (scaled < -32768.0f) return -32768;
        return static_cast<int16_t>(scaled);
    }

    void TraceLogger::reset(vbt::ActivityMode mode)
    {
        frozen_ = true;
        mode_ = mode;
        writeIndex_ = 0;
        count_ = 0;
        startTimestampUs_ = 0;
        decimCounter_ = 0;
        lastPhase_ = 0;
        frozen_ = false;
    }

    void TraceLogger::record(const vbt::KinematicSample& sample, uint8_t phase, bool stationary, bool completed)
    {
        if (frozen_) return;

        const bool stateChanged = (count_ > 0 && phase != lastPhase_) || completed;
        if (!stateChanged && (++decimCounter_ < DECIMATION)) return;
        decimCounter_ = 0;

        if (startTimestampUs_ == 0)
        {
            startTimestampUs_ = sample.timestamp_us;
        }

        const uint64_t elapsedUs = sample.timestamp_us >= startTimestampUs_
            ? (sample.timestamp_us - startTimestampUs_)
            : 0;

        TraceSample entry{};
        entry.dt_ms    = static_cast<uint32_t>(elapsedUs / 1000ULL);
        entry.az_cmps2 = clampInt16(sample.linear_acceleration_mps2.z * 100.0f);
        entry.vz_mmps  = clampInt16(sample.velocity_mps.z * 1000.0f);
        entry.pz_mm    = clampInt16(sample.position_m.z * 1000.0f);
        entry.phase    = phase;
        entry.flags    = (stationary ? FLAG_STATIONARY : 0u) | (completed ? FLAG_COMPLETED : 0u);

        buffer_[writeIndex_] = entry;
        writeIndex_ = (writeIndex_ + 1) % CAPACITY;
        if (count_ < CAPACITY) ++count_;
        lastPhase_ = phase;
    }

    void TraceLogger::freeze()
    {
        frozen_ = true;
    }

    bool TraceLogger::sampleAt(size_t logicalIndex, TraceSample& out) const
    {
        if (!frozen_ || logicalIndex >= count_) return false;
        const size_t oldestIndex = (count_ == CAPACITY) ? writeIndex_ : 0;
        const size_t physicalIndex = (oldestIndex + logicalIndex) % CAPACITY;
        out = buffer_[physicalIndex];
        return true;
    }

    TraceLogger& traceLogger()
    {
        static TraceLogger instance{};
        return instance;
    }
}
