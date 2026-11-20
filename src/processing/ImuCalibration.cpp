#include "processing/ImuCalibration.h"

#include <cmath>

namespace
{
    constexpr float GRAVITY_MPS2 = 9.80665f;

    // Stationarity thresholds. These are initial engineering limits,
    // to be validated against real sensor measurements.
    constexpr float MAX_MEAN_GYRO_RADPS = 0.15f;
    constexpr float MAX_GYRO_STDDEV_RADPS = 0.02f;
    constexpr float MAX_ACCEL_STDDEV_MPS2 = 0.15f;
    constexpr float MIN_GRAVITY_MAGNITUDE = GRAVITY_MPS2 * 0.85f;
    constexpr float MAX_GRAVITY_MAGNITUDE = GRAVITY_MPS2 * 1.15f;

    bool finiteVector(const vbt::Vector3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    vbt::Vector3 calculateMean(const vbt::Vector3& sum, uint32_t count)
    {
        const float divisor = static_cast<float>(count);

        return { sum.x / divisor, sum.y / divisor, sum.z / divisor };
    }

    vbt::Vector3 calculateStdDev(const vbt::Vector3& sum, const vbt::Vector3& squaredSum, uint32_t count)
    {
        const float divisor = static_cast<float>(count);
        const vbt::Vector3 mean = calculateMean(sum, count);

        const float varianceX = (squaredSum.x / divisor) - mean.x * mean.x;
        const float varianceY = (squaredSum.y / divisor) - mean.y * mean.y;
        const float varianceZ = (squaredSum.z / divisor) - mean.z * mean.z;

        return { std::sqrt(varianceX > 0.0f ? varianceX : 0.0f), std::sqrt(varianceY > 0.0f ? varianceY : 0.0f), std::sqrt(varianceZ > 0.0f ? varianceZ : 0.0f) };
    }

    float maximumComponent(const vbt::Vector3& value)
    {
        float result = value.x;

        if(value.y > result) result = value.y;
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
        const SampleStatus required = SampleStatus::Valid | SampleStatus::AccelorometerOk | SampleStatus::GyroscopeOk | SampleStatus::TimestampValid | SampleStatus::SequenceValid;

        const uint32_t requiredBits = toBits(required);
        const uint32_t actualBits = toBits(sample.status);
        return(actualBits & requiredBits) == requiredBits && finiteVector(sample.accelerometer) && finiteVector(sample.gyroscope);
    }

    bool ImuCalibrator::collect(const RawImuSample& sample)
    {
        if(state_ != State::Collecting) return state_ == State::Ready;
        if(!isValidRawSample(sample)) return false;

        const Vector3& gyro = sample.gyroscope;
        const Vector3& accel = sample.accelerometer;

        gyroSum_.x += gyro.x;
        gyroSum_.y += gyro.y;
        gyroSum_.z += gyro.z;

        gyroSquaredSum_.x += gyro.x * gyro.x;
        gyroSquaredSum_.y += gyro.y * gyro.y;
        gyroSquaredSum_.z += gyro.z * gyro.z;

        accelSum_.x += accel.x;
        accelSum_.y += accel.y;
        accelSum_.z += accel.z;

        accelSquaredSum_.x += accel.x * accel.x;
        accelSquaredSum_.y += accel.y * accel.y;
        accelSquaredSum_.z += accel.z * accel.z;

        ++sampleCount_;

        if(sampleCount_ < CALIBRATION_SAMPLE_COUNT) return false;

        if(!validateStationarity())
        {
            state_ = State::Failed;
            return false;
        }

        gyroBiasRadps_ = calculateMean(gyroSum_, sampleCount_);

        // This is the measured stationary acceleration vector,
        // including gravity. It is NOT an independently measured
        // accelerometer bias.
        gravityReferenceMps2_ = calculateMean(accelSum_, sampleCount_);

        state_ = State::Ready;
        return true;
    }

    bool ImuCalibrator::validateStationarity() const
    {
        if(sampleCount_ == 0) return false;

        const Vector3 meanGyro = calculateMean(gyroSum_, sampleCount_);
        const Vector3 meanAccel = calculateMean(accelSum_, sampleCount_);

        const Vector3 gyroStdDev = calculateStdDev(gyroSum_, gyroSquaredSum_, sampleCount_);
        const Vector3 accelStdDev = calculateStdDev( accelSum_, accelSquaredSum_, sampleCount_);

        const float gyroMagnitude = meanGyro.magnitude();
        const float gravityMagnitude = meanAccel.magnitude();

        if (gyroMagnitude > MAX_MEAN_GYRO_RADPS) return false;
        if (maximumComponent(gyroStdDev) > MAX_GYRO_STDDEV_RADPS) return false;
        if (maximumComponent(accelStdDev) > MAX_ACCEL_STDDEV_MPS2) return false;
        if (gravityMagnitude < MIN_GRAVITY_MAGNITUDE || gravityMagnitude > MAX_GRAVITY_MAGNITUDE)return false;

        return true;

    }

    bool ImuCalibrator::apply( const RawImuSample& raw, CalibratedImuSample& calibrated) const
    {
        if(state_ != State::Ready || !isValidRawSample(raw)) return false;

        calibrated.timestamp_us = raw.timestamp_us;
        calibrated.sequence = raw.sequence;

        // Preserve acceleration in the body frame.
        // Gravity removal requires orientation compensation later.
        calibrated.accelerometer_mps2 = raw.accelerometer;
        calibrated.gyroscope_radps = { raw.gyroscope.x - gyroBiasRadps_.x, raw.gyroscope.y - gyroBiasRadps_.y, raw.gyroscope.z - gyroBiasRadps_.z };
        calibrated.status = raw.status | SampleStatus::Calibrated;

        return true;
    }
  
}

