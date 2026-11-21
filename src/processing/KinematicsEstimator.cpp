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
        initialized_ = false;
        stationary_ = false;
        previousTimestampUs_ = 0;
        stationaryCount_ = 0;
        previousLinearAccelerationMps2_ = {};
        velocityMps_ = {};
        positionM_ = {};
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

    bool KinematicsEstimator::isSampleQuiet(const Vector3& linearAccelerationMps2, const Vector3& gyroscopeRadps)
    {
        return linearAccelerationMps2.magnitude() <= STATIONARY_ACCEL_THRESHOLD_MPS2 && gyroscopeRadps.magnitude() <= STATIONARY_GYRO_THRESHOLD_RADPS;
    }

    Vector3 KinematicsEstimator::integrateTrapezoid(const Vector3& accumulated, const Vector3& previousRate, const Vector3& currentRate, float dt)
    {
        const float halfDt = 0.5f * dt;
        return { accumulated.x + halfDt * (previousRate.x + currentRate.x), accumulated.y + halfDt * (previousRate.y + currentRate.y), accumulated.z + halfDt * (previousRate.z + currentRate.z) };
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

        const bool quiet = isSampleQuiet(linearAcceleration, imuSample.gyroscope_radps);

        if(!initialized_)
        {
            previousTimestampUs_ = imuSample.timestamp_us;
            previousLinearAccelerationMps2_ = linearAcceleration;
            velocityMps_ = {};
            positionM_ = {};
            stationaryCount_ = quiet ? 1 : 0;
            stationary_ = quiet;
            initialized_ = true;
        }
        else
        {
            if(imuSample.timestamp_us <= previousTimestampUs_) return false;

            const uint64_t intervalUs = imuSample.timestamp_us - previousTimestampUs_;
            previousTimestampUs_ = imuSample.timestamp_us;

            if(intervalUs > MAX_SAMPLE_INTERVAL_US)
            {
                // Stale gap: reset velocity integration to avoid a huge jump.
                velocityMps_ = {};
                stationaryCount_ = quiet ? 1 : 0;
                stationary_ = quiet;
            }
            else
            {
                if (quiet) 
                {
                   if (stationaryCount_ < STATIONARY_HOLD_SAMPLES)  ++stationaryCount_;
                } 
                else 
                {
                    stationaryCount_ = 0;
                }

                stationary_ = (stationaryCount_ >= STATIONARY_HOLD_SAMPLES);

                if (stationary_)
                {
                    // Zero-Velocity Update (ZUPT): clamp velocity drift when at rest.
                    velocityMps_ = {};
                }
                else
                {
                    const float dt = static_cast<float>(intervalUs) / 1000000.0f;
                    const Vector3 previousVelocity = velocityMps_;
                    velocityMps_ = integrateTrapezoid( previousVelocity, previousLinearAccelerationMps2_, linearAcceleration, dt);
                    positionM_ = integrateTrapezoid( positionM_, previousVelocity, velocityMps_, dt);
                }
            }

            previousLinearAccelerationMps2_ = linearAcceleration;
        }

        if (!finiteVector(velocityMps_) || !finiteVector(positionM_)) return false;

        SampleStatus status = orientationState.status | SampleStatus::KinematicsValid;
        if (stationary_) status = status | SampleStatus::Stationary;
        
        output.timestamp_us = imuSample.timestamp_us;
        output.sequence = imuSample.sequence;
        output.linear_acceleration_mps2 = linearAcceleration;
        output.velocity_mps = velocityMps_;
        output.position_m = positionM_;
        output.status = status;

        return true;
    }
}