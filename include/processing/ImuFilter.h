#pragma once

#include "contracts/ImuTypes.h"

namespace vbt::processing
{
    class ImuFilter
    {
        public:
            static constexpr float CUTOFF_HZ = 20.0f;
            static constexpr uint64_t MAX_SAMPLE_INTERVAL_US = 20000;

            bool process (const CalibratedImuSample& input, FilteredImuSample& output);

            void reset();
        
        private:
            bool initialized_{false};
            uint64_t previousTimestampUs_{0};
            Vector3 filteredAccelerometer_{};
            Vector3 filteredGyroscope_{};

            static Vector3 lowPass(const Vector3& previous, const Vector3& input, float alpha);

    };
}