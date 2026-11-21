#include "processing/kinematicsEstimator.h"

#include <cmath>

namespace
{
    bool finiteVector(const vbt::Vector3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    bool finiteQuaternion(const vbt::Quaternion q)
    {
        return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
    }

    bool isNormalizedQuaternion(const vbt::Quaternion& q)
    {
        const float normSq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
        return normSq >= 0.99f && normSq <= 1.01f;
    }

    vbt::Vector3 cross(const vbt::Vector3& a, const vbt::Vector3& b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }
}

namespace vbt::processing
{
    void KinematicsEstimator::reset()
    {
         // Ticket 2 will reset velocity, position, and stationary timer state here.
    }

    Vector3 KinematicsEstimator::rotateBodyToWorld(const Vector3& bodyVector, const Quaternion& bodyToWorld)
    {
        // Fast quaternion-vector rotation:
        // t = 2 * cross(q.xyz, v)
        // v_world = v + q.w * t + cross(q.xyz, t)
        const Vector3 qVec{bodyToWorld.x, bodyToWorld.y, bodyToWorld.z};
        const Vector3 c1 = cross(qVec, bodyVector);
        const Vector3 t{2.0f * c1.x, 2.0f * c1.y, 2.0f * c1.z};
        const Vector3 c2 = cross(qVec, t);

        return { bodyVector.x + bodyToWorld.w * t.x + c2.x, bodyVector.y + bodyToWorld.w * t.y + c2.y, bodyVector.z + bodyToWorld.w * t.z + c2.z };
    }

    Vector3 KinematicsEstimator::removeGravity(const Vector3& worldAccelerationMps2)
    {
        return { worldAccelerationMps2.x, worldAccelerationMps2.y, worldAccelerationMps2.z - GRAVITY_MPS2};
    }

    bool KinematicsEstimator::update(const FilteredImuSample& imuSample, const OrientationState& orientationState, KinematicSample& output)
    {
        const uint32_t requiredImuBits = toBits(SampleStatus::Valid) | toBits(SampleStatus::AccelorometerOk) | toBits(SampleStatus::GyroscopeOk) | toBits(SampleStatus::TimestampValid) | toBits(SampleStatus::SequenceValid) | toBits(SampleStatus::Calibrated) | toBits(SampleStatus::Filtered);

        const uint32_t requiredOrientationBits = requiredImuBits | toBits(SampleStatus::OrientationValid);

        if ((toBits(imuSample.status) & requiredImuBits) != requiredImuBits) return false;
        if ((toBits(orientationState.status) & requiredOrientationBits) != requiredOrientationBits) return false;

        // Ensure the IMU sample and orientation state belong to the exact same tick.
        if(imuSample.timestamp_us != orientationState.timestamp_us || imuSample.sequence != orientationState.sequence) return false;
        if (!finiteVector(imuSample.accelerometer_mps2) || !finiteVector(imuSample.gyroscope_radps) || !finiteQuaternion(orientationState.body_to_world) || !isNormalizedQuaternion(orientationState.body_to_world)) return false;
        
        const Vector3 worldAcceleration = rotateBodyToWorld(imuSample.accelerometer_mps2, orientationState.body_to_world);
        const Vector3  linearAcceleration = removeGravity(worldAcceleration);

        if (!finiteVector(linearAcceleration)) return false;

        output.timestamp_us = imuSample.timestamp_us;
        output.sequence = imuSample.sequence;
        output.linear_acceleration_mps2 = linearAcceleration;
        output.velocity_mps = {};
        output.position_m = {};
        output.status = orientationState.status | SampleStatus::KinematicsValid;

        return true;
    }
}