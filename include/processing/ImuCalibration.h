#pragma once

#include <cstdint>
#include "contracts/ImuTypes.h"

namespace vbt::processing
{
    class ImuCalibrator
    {
    public:
        static constexpr uint32_t CALIBRATION_SAMPLE_COUNT = 500;

        enum class State : uint8_t
        {
            Collecting,
            Ready,
            Failed
        };

        // Feed valid stationary samples until calibration completes.
        // Returns true when calibration has completed successfully.
        bool collect(const RawImuSample& sample);

        // Convert a raw sample using the estimated gyro bias.
        // Returns false until calibration is ready or if input is invalid.
        bool apply(const RawImuSample& raw, CalibratedImuSample& calibrated) const;

        void reset();

        State state() const { return state_; }
        const Vector3& gyroscopeBiasRadps() const { return gyroBiasRadps_; }
        const Vector3& gravityReferenceMps2() const { return gravityReferenceMps2_; }
        float calibratedGravityMagnitudeMps2() const { return gravityReferenceMps2_.magnitude(); }
        uint32_t collectedSamples() const { return sampleCount_; }

    private:
        struct DoubleVec3
        {
            double x{0.0};
            double y{0.0};
            double z{0.0};
        };

        bool isValidRawSample(const RawImuSample& sample) const;
        bool validateStationarity() const;

        State state_{State::Collecting};
        uint32_t sampleCount_{0};
        DoubleVec3 gyroSum_{};
        DoubleVec3 gyroSquaredSum_{};
        DoubleVec3 accelSum_{};
        DoubleVec3 accelSquaredSum_{};
        Vector3 gyroBiasRadps_{};
        Vector3 gravityReferenceMps2_{};
    };
}
