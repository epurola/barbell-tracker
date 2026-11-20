#include "processing/OrientationEstimator.h"
#include <cmath>

namespace
{
    bool finiteVector(const vbt::Vector3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    bool finiteQuaternion(const vbt::Quaternion& q)
    {
        return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
    }

     vbt::Vector3 normalizeVector(const vbt::Vector3& value)
    {
        const float magnitude = value.magnitude();

        if (magnitude <= 0.000001f) return {0.0f, 0.0f, 1.0f};

        return { value.x / magnitude, value.y / magnitude, value.z / magnitude };
    }

    vbt::Vector3 cross(const vbt::Vector3& a, const vbt::Vector3& b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }
}

namespace vbt::processing
{
    void OrientationEstimator::reset()
    {
        initialized_ = false;
        previousTimestampUs_ = 0;
        orientation_ = {};
    }

    Quaternion OrientationEstimator::normalize(const Quaternion& q)
    {
        const float norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        if (norm <= 0.000001f || !std::isfinite(norm)) return {};
        
        return {q.w / norm, q.x / norm, q.y / norm, q.z / norm };
    }

    Quaternion OrientationEstimator::initializeFromGravity(const Vector3& accelerationMps2)
    {

        // Shortest-arc rotation from measured body up-vector (gravity reaction)
        // to world up-vector (0, 0, 1).
        const Vector3 bodyUp = normalizeVector(accelerationMps2);

        // dot(bodyUp, worldUp) = bodyUp.z
        // cross(bodyUp, worldUp) = (bodyUp.y, -bodyUp.x, 0)
        const float dot = bodyUp.z;
        if (dot < -0.9999f) return {0.0f, 1.0f, 0.0f, 0.0f}; // Sensor is upside down: 180-degree rotation around X axis.
    
        const Quaternion q{1.0f + dot, bodyUp.y, -bodyUp.x,  0.0f };

        return normalize(q);
    }

    Vector3 OrientationEstimator::expectedGravityBody(const Quaternion& bodyToWorld)
    {
        // Rotate world up-vector (0, 0, 1) into body frame using inverse (conjugate) of bodyToWorld.
        const float w = bodyToWorld.w;
        const float x = bodyToWorld.x;
        const float y = bodyToWorld.y;
        const float z = bodyToWorld.z;
        return {2.0f * (x * z - w * y), 2.0f * (w * x + y * z), w * w - x * x - y * y + z * z };
    }

     bool OrientationEstimator::update(const FilteredImuSample& input, OrientationState& output)
    {
        const uint32_t requiredBits = toBits(SampleStatus::Valid) | toBits(SampleStatus::AccelorometerOk) | toBits(SampleStatus::GyroscopeOk) | toBits(SampleStatus::TimestampValid) | toBits(SampleStatus::SequenceValid) | toBits(SampleStatus::Calibrated) | toBits(SampleStatus::Filtered);
        
        if ((toBits(input.status) & requiredBits) != requiredBits) return false;
        if (!finiteVector(input.accelerometer_mps2) || !finiteVector(input.gyroscope_radps)) return false;

        const float accelMagnitude = input.accelerometer_mps2.magnitude();
        if (accelMagnitude <= 0.0001f) return false;

        if (!initialized_)
        {
            orientation_ = initializeFromGravity(input.accelerometer_mps2);
            previousTimestampUs_ = input.timestamp_us;
            initialized_ = true;
        }
        else
        {
            if (input.timestamp_us <= previousTimestampUs_) return false;

            const uint64_t intervalUs = input.timestamp_us - previousTimestampUs_;
            previousTimestampUs_ = input.timestamp_us;

            if (intervalUs > MAX_SAMPLE_INTERVAL_US)
            {
                orientation_ = initializeFromGravity(input.accelerometer_mps2);
            }
            else
            {
                const float dt = static_cast<float>(intervalUs) / 1000000.0f;
                Vector3 correctedGyro = input.gyroscope_radps;

                // Apply accelerometer roll/pitch correction only when total
                // acceleration magnitude is close to 1g (minimal linear acceleration).
                if (accelMagnitude >= MIN_CORRECTION_GRAVITY_MPS2 && accelMagnitude <= MAX_CORRECTION_GRAVITY_MPS2)
                {
                    const Vector3 measuredUp = normalizeVector(input.accelerometer_mps2);
                    const Vector3 estimatedUp = expectedGravityBody(orientation_);
                    const Vector3 error = cross(measuredUp, estimatedUp);
                    correctedGyro.x += ACCEL_CORRECTION_GAIN * error.x;
                    correctedGyro.y += ACCEL_CORRECTION_GAIN * error.y;
                    correctedGyro.z += ACCEL_CORRECTION_GAIN * error.z;
                }

                const Quaternion& q = orientation_;
                const float halfDt = 0.5f * dt;
                
                // Quaternion derivative: q_dot = 0.5 * q * [0, wx, wy, wz]
                const Quaternion integrated{
                    q.w + halfDt * (-q.x * correctedGyro.x - q.y * correctedGyro.y - q.z * correctedGyro.z),
                    q.x + halfDt * ( q.w * correctedGyro.x + q.y * correctedGyro.z - q.z * correctedGyro.y),
                    q.y + halfDt * ( q.w * correctedGyro.y - q.x * correctedGyro.z + q.z * correctedGyro.x),
                    q.z + halfDt * ( q.w * correctedGyro.z + q.x * correctedGyro.y - q.y * correctedGyro.x)
                };

                if (!finiteQuaternion(integrated)) return false;

                orientation_ = normalize(integrated);
            }
        }

        output.timestamp_us = input.timestamp_us;
        output.sequence = input.sequence;
        output.body_to_world = orientation_;
        output.status = input.status | SampleStatus::OrientationValid;
        
        return true;
    }


}