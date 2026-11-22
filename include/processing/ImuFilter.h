#pragma once

#include "contracts/ImuTypes.h"

namespace vbt::processing
{
    class ImuFilter
    {
    public:
        static constexpr float CUTOFF_HZ = 20.0f;
        static constexpr float HIGH_SPEED_CUTOFF_HZ = 50.0f;
        static constexpr uint64_t MAX_SAMPLE_INTERVAL_US = 20000;

        bool process(const CalibratedImuSample& input, FilteredImuSample& output);

        void reset();

        void setCutoffHz(float cutoffHz)
        {
            if (cutoffHz >= 1.0f && cutoffHz <= 200.0f)
            {
                cutoffHz_ = cutoffHz;
            }
        }

        float cutoffHz() const { return cutoffHz_; }

    private:
        bool initialized_{false};
        float cutoffHz_{CUTOFF_HZ};
        uint64_t previousTimestampUs_{0};
        Vector3 filteredAccelerometer_{};
        Vector3 filteredGyroscope_{};

        static Vector3 lowPass(const Vector3& previous, const Vector3& input, float alpha);
    };
}
