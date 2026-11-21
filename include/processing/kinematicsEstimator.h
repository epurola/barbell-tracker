#pragma once

#include "contracts/FirmwareContracts.h"

namespace vbt::processing
{
    class KinematicsEstimator
    {
        public:
            static constexpr float GRAVITY_MPS2 = 9.80665f;
            static constexpr float STATIONARY_ACCEL_THRESHOLD_MPS2 = 0.25f;
            static constexpr float STATIONARY_GYRO_THRESHOLD_RADPS = 0.10f;
            static constexpr uint32_t STATIONARY_HOLD_SAMPLES = 50; // 100 ms at 500 Hz
            static constexpr uint64_t MAX_SAMPLE_INTERVAL_US = 50000;

            bool update(const FilteredImuSample& imuSample, const OrientationState& OrientationState, KinematicSample& output);
            void reset();
            static Vector3 rotateBodyToWorld(const Vector3& bodyVector, const Quaternion& bodyToWorld);
            static Vector3 removeGravity(const Vector3& worldAccelerationMps2);
            bool initialized() const { return initialized_; }
            bool isStationary() const { return stationary_; }
            const Vector3& velocityMps() const { return velocityMps_; }
            const Vector3& positionM() const { return positionM_; }

        private:
        bool initialized_{false};
        bool stationary_{false};
        uint64_t previousTimestampUs_{0};
        uint32_t stationaryCount_{0};
        Vector3 previousLinearAccelerationMps2_{};
        Vector3 velocityMps_{};
        Vector3 positionM_{};
        static bool isSampleQuiet(const Vector3& linearAccelerationMps2, const Vector3& gyroscopeRadps);
        static Vector3 integrateTrapezoid(const Vector3& accumulated, const Vector3& previousRate, const Vector3& currentRate, float dt);
    };
}

