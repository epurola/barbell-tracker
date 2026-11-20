#pragma once
#include <cstdint>
#include "contracts/ImuTypes.h"
#include "contracts/OrientationTypes.h"

namespace vbt::processing
{
    class OrientationEstimator
    {
        public:
            static constexpr float GRAVITY_MPS2 = 9.80665f;
            static constexpr float MIN_CORRECTION_GRAVITY_MPS2 = GRAVITY_MPS2 * 0.85f;
            static constexpr float MAX_CORRECTION_GRAVITY_MPS2 = GRAVITY_MPS2 * 1.15f;
            static constexpr float ACCEL_CORRECTION_GAIN = 1.5f;
            static constexpr uint64_t MAX_SAMPLE_INTERVAL_US = 50000;

        bool update(const FilteredImuSample& input, OrientationState& output);
        void reset();
        bool initialized() const { return initialized_; }
        const Quaternion& orientation() const { return orientation_; }

        private:
            bool initialized_{false};
            uint64_t previousTimestampUs_{0};
            Quaternion orientation_{};
            static Quaternion initializeFromGravity(const Vector3& accelerationMps2);
            static Quaternion normalize(const Quaternion& q);
            static Vector3 expectedGravityBody(const Quaternion& bodyToWorld);
    };
}