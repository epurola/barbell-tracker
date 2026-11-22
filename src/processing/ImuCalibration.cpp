#include "processing/ImuCalibration.h"

#include <cmath>

namespace
{
    constexpr float GRAVITY_MPS2 = 9.80665f;

    // Stationarity thresholds.
    constexpr float MAX_MEAN_GYRO_RADPS = 0.15f;
    constexpr float MAX_GYRO_STDDEV_RADPS = 0.02f;
    constexpr float MAX_ACCEL_STDDEV_MPS2 = 0.15f;
    constexpr float MIN_GRAVITY_MAGNITUDE = GRAVITY_MPS2 * 0.85f;
    constexpr float MAX_GRAVITY_MAGNITUDE = GRAVITY_MPS2 * 1.15f;

    bool finiteVector(const vbt::Vector3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    float maximumComponent(const vbt::Vector3& value)
    {
        float result = value.x;
        if (value.y > result) result = value.y;
        if (value.z > result) result = value.z;
        return result;
    }
}

namespace vbt::processing
{
    void ImuCalibrator::reset()
    {
        state_ = State::Collecting;
        sampleCount_ = 0;

        gyroSum_ = {};
        gyroSquaredSum_ = {};

        accelSum_ = {};
        accelSquaredSum_ = {};

        gyroBiasRadps_ = {};
        gravityReferenceMps2_ = {};
    }

    bool ImuCalibrator::isValidRawSample(const RawImuSample& sample) const
    {
        const SampleStatus required =
            SampleStatus::Valid |
            SampleStatus::AccelorometerOk |
            SampleStatus::GyroscopeOk |
            SampleStatus::TimestampValid |
            SampleStatus::SequenceValid;

        const uint32_t requiredBits = toBits(required);
        const uint32_t actualBits = toBits(sample.status);
        return (actualBits & requiredBits) == requiredBits &&
               finiteVector(sample.accelerometer) &&
               finiteVector(sample.gyroscope);
    }

    bool ImuCalibrator::collect(const RawImuSample& sample)
    {
        if (state_ != State::Collecting) return state_ == State::Ready;
        if (!isValidRawSample(sample)) return false;

        const double gx = static_cast<double>(sample.gyroscope.x);
        const double gy = static_cast<double>(sample.gyroscope.y);
        const double gz = static_cast<double>(sample.gyroscope.z);

        const double ax = static_cast<double>(sample.accelerometer.x);
        const double ay = static_cast<double>(sample.accelerometer.y);
        const double az = static_cast<double>(sample.accelerometer.z);

        gyroSum_.x += gx;
        gyroSum_.y += gy;
        gyroSum_.z += gz;

        gyroSquaredSum_.x += gx * gx;
        gyroSquaredSum_.y += gy * gy;
        gyroSquaredSum_.z += gz * gz;

        accelSum_.x += ax;
        accelSum_.y += ay;
        accelSum_.z += az;

        accelSquaredSum_.x += ax * ax;
        accelSquaredSum_.y += ay * ay;
        accelSquaredSum_.z += az * az;

        ++sampleCount_;

        if (sampleCount_ < CALIBRATION_SAMPLE_COUNT) return false;

        if (!validateStationarity())
        {
            state_ = State::Failed;
            return false;
        }

        const double invCount = 1.0 / static_cast<double>(sampleCount_);

        gyroBiasRadps_ = {
            static_cast<float>(gyroSum_.x * invCount),
            static_cast<float>(gyroSum_.y * invCount),
            static_cast<float>(gyroSum_.z * invCount)
        };

        gravityReferenceMps2_ = {
            static_cast<float>(accelSum_.x * invCount),
            static_cast<float>(accelSum_.y * invCount),
            static_cast<float>(accelSum_.z * invCount)
        };

        state_ = State::Ready;
        return true;
    }

    bool ImuCalibrator::validateStationarity() const
    {
        if (sampleCount_ == 0) return false;

        const double invCount = 1.0 / static_cast<double>(sampleCount_);

        const double meanGx = gyroSum_.x * invCount;
        const double meanGy = gyroSum_.y * invCount;
        const double meanGz = gyroSum_.z * invCount;

        const double meanAx = accelSum_.x * invCount;
        const double meanAy = accelSum_.y * invCount;
        const double meanAz = accelSum_.z * invCount;

        const double varGx = (gyroSquaredSum_.x * invCount) - (meanGx * meanGx);
        const double varGy = (gyroSquaredSum_.y * invCount) - (meanGy * meanGy);
        const double varGz = (gyroSquaredSum_.z * invCount) - (meanGz * meanGz);

        const double varAx = (accelSquaredSum_.x * invCount) - (meanAx * meanAx);
        const double varAy = (accelSquaredSum_.y * invCount) - (meanAy * meanAy);
        const double varAz = (accelSquaredSum_.z * invCount) - (meanAz * meanAz);

        const Vector3 meanGyro{
            static_cast<float>(meanGx),
            static_cast<float>(meanGy),
            static_cast<float>(meanGz)
        };
        const Vector3 meanAccel{
            static_cast<float>(meanAx),
            static_cast<float>(meanAy),
            static_cast<float>(meanAz)
        };

        const Vector3 gyroStdDev{
            static_cast<float>(std::sqrt(varGx > 0.0 ? varGx : 0.0)),
            static_cast<float>(std::sqrt(varGy > 0.0 ? varGy : 0.0)),
            static_cast<float>(std::sqrt(varGz > 0.0 ? varGz : 0.0))
        };
        const Vector3 accelStdDev{
            static_cast<float>(std::sqrt(varAx > 0.0 ? varAx : 0.0)),
            static_cast<float>(std::sqrt(varAy > 0.0 ? varAy : 0.0)),
            static_cast<float>(std::sqrt(varAz > 0.0 ? varAz : 0.0))
        };

        const float gyroMagnitude = meanGyro.magnitude();
        const float gravityMagnitude = meanAccel.magnitude();

        if (gyroMagnitude > MAX_MEAN_GYRO_RADPS) return false;
        if (maximumComponent(gyroStdDev) > MAX_GYRO_STDDEV_RADPS) return false;
        if (maximumComponent(accelStdDev) > MAX_ACCEL_STDDEV_MPS2) return false;
        if (gravityMagnitude < MIN_GRAVITY_MAGNITUDE || gravityMagnitude > MAX_GRAVITY_MAGNITUDE) return false;

        return true;
    }

    bool ImuCalibrator::apply(const RawImuSample& raw, CalibratedImuSample& calibrated) const
    {
        if (state_ != State::Ready || !isValidRawSample(raw)) return false;

        calibrated.timestamp_us = raw.timestamp_us;
        calibrated.sequence = raw.sequence;

        // Preserve acceleration in the body frame.
        // Gravity removal requires orientation compensation later.
        calibrated.accelerometer_mps2 = raw.accelerometer;
        calibrated.gyroscope_radps = {
            raw.gyroscope.x - gyroBiasRadps_.x,
            raw.gyroscope.y - gyroBiasRadps_.y,
            raw.gyroscope.z - gyroBiasRadps_.z
        };
        calibrated.status = raw.status | SampleStatus::Calibrated;

        return true;
    }
}
