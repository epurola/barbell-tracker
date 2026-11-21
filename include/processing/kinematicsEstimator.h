#pragma once

#include "contracts/FirmwareContracts.h"

namespace vbt::processing
{
    class KinematicsEstimator
    {
        public:
            static constexpr float GRAVITY_MPS2 = 9.80665f;

            bool update(const FilteredImuSample& imuSample, const OrientationState& OrientationState, KinematicSample& output);
            void reset();
            static Vector3 rotateBodyToWorld(const Vector3& bodyVector, const Quaternion& bodyToWorld);

            static Vector3 removeGravity(const Vector3& worldAccelerationMps2);
    };
}

