#include "processing/ImuFilter.h"

#include <cmath>

namespace
{
    bool finiteVector(const vbt::Vector3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }
}

namespace vbt::processing
{
    void ImuFilter::reset()
    {
        initialized_ = false;
        previousTimestampUs_ = 0;

        filteredAccelerometer_ = {};
        filteredGyroscope_ = {};
    }

    Vector3 ImuFilter::lowPass(const Vector3& previous, const Vector3& input, float alpha)
    {
        return { previous.x + alpha * (input.x - previous.x), previous.y + alpha * (input.y - previous.y),previous.z + alpha * (input.z - previous.z) };
    }

    bool ImuFilter::process(const CalibratedImuSample& input, FilteredImuSample& output)
    {
        const uint32_t requiredBits = toBits(SampleStatus::Valid) | toBits(SampleStatus::AccelorometerOk) | toBits(SampleStatus::GyroscopeOk) | toBits(SampleStatus::TimestampValid) | toBits(SampleStatus::SequenceValid) | toBits(SampleStatus::Calibrated);

        if ((toBits(input.status) & requiredBits) != requiredBits) return false;
        if (!finiteVector(input.accelerometer_mps2) || !finiteVector(input.gyroscope_radps)) return false;

        // First sample initializes the filter.
        if (!initialized_)
        {
            filteredAccelerometer_ = input.accelerometer_mps2;
            filteredGyroscope_ = input.gyroscope_radps;

            previousTimestampUs_ = input.timestamp_us;
            initialized_ = true;
        }
        else
        {
            if(input.timestamp_us <= previousTimestampUs_) return false;
            const uint64_t intervalUs = input.timestamp_us - previousTimestampUs_;

            // A large gap means the filter's previous state is stale.
            // Reset and seed it with the current valid sample.
            if (intervalUs > MAX_SAMPLE_INTERVAL_US)
            {
                filteredAccelerometer_ = input.accelerometer_mps2;
                filteredGyroscope_ = input.gyroscope_radps;
            }
            else
            {
                const float dt = static_cast<float>(intervalUs) / 1000000.0f;
                constexpr float TWO_PI = 6.28318530718f;
                const float rc = 1.0f / (TWO_PI * CUTOFF_HZ);
                const float alpha = dt / (rc + dt);

                filteredAccelerometer_ = lowPass( filteredAccelerometer_, input.accelerometer_mps2, alpha);
                filteredGyroscope_ = lowPass(filteredGyroscope_, input.gyroscope_radps, alpha);
            }

            previousTimestampUs_ = input.timestamp_us;
        }

        output.timestamp_us = input.timestamp_us;
        output.sequence = input.sequence;

        output.accelerometer_mps2 = filteredAccelerometer_;
        output.gyroscope_radps = filteredGyroscope_;

        output.status = input.status | SampleStatus::Filtered;

        return true;

    }


}