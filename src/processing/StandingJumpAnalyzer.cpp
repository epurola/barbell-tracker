#include "processing/StandingJumpanalyzer.h"

namespace vbt::processing
{
    void StandingJumpAnalyzer::onReset()
    {
        resetCurrentJump();
        armed_ = false;
        jumpCount_ = 0;
        jumpHeightSumCm_ = 0.0f;
        lastCompletedJump_ = {};
    }

    void StandingJumpAnalyzer::resetCurrentJump()
    {
        phase_ = Phase::Idle;
        takeoffTimestampUs_ = 0;
        propulsionStartTimestampUs_ = 0;
        peakVelocityMps_ = 0.0f;
        takeoffVelocityMps_ = 0.0f;
    }

    bool StandingJumpAnalyzer::onFinishRecording(ActivityOutput& output)
    {
        resetCurrentJump();
        if (jumpCount_ > 0)
        {
            output.jump = lastCompletedJump_;
            return true;
        }
        return false;
    }

    bool StandingJumpAnalyzer::onSample(const KinematicSample& sample, bool stationary, ActivityOutput& output)
    {
        const float vz = sample.velocity_mps.z;
        const float az = sample.linear_acceleration_mps2.z;

        switch (phase_)
        {
            case Phase::Idle:
                HandleIdle(vz, az, sample.timestamp_us, stationary);
                break;

            case Phase::Propulsion:
                HandlePropulsion(vz, az, sample.timestamp_us, stationary);
                break;

            case Phase::Flight:
                return HandleFlight(az, sample.timestamp_us, output.jump);

            default:
                resetCurrentJump();
                break;
        }

        output.jump = lastCompletedJump_;
        return false;
    }

    void StandingJumpAnalyzer::HandleIdle(float vz, float az, uint64_t timestampUs, bool stationary)
    {
        if (stationary)
        {
            armed_ = true;
            return;
        }

        if (!armed_) return;

        if (vz <= DIP_START_VELOCITY_MPS || az >= PUSH_START_ACCEL_MPS2)
        {
            phase_ = Phase::Propulsion;
            propulsionStartTimestampUs_ = timestampUs;
            peakVelocityMps_ = vz > 0.0f ? vz : 0.0f;
            takeoffVelocityMps_ = 0.0f;
        }
    }

    void StandingJumpAnalyzer::HandlePropulsion(float vz, float az, uint64_t timestampUs, bool stationary)
    {
        if (timestampUs - propulsionStartTimestampUs_ > MAX_PROPULSION_DURATION_US)
        {
            resetCurrentJump();
            armed_ = false;
            return;
        }

        if (vz > peakVelocityMps_) peakVelocityMps_ = vz;

        if (stationary)
        {
            resetCurrentJump();
            armed_ = true;
            return;
        }

        if (az <= TAKEOFF_FREEFALL_ACCEL_MPS2 && peakVelocityMps_ >= MIN_TAKEOFF_VELOCITY_MPS)
        {
            phase_ = Phase::Flight;
            takeoffTimestampUs_ = timestampUs;
            takeoffVelocityMps_ = vz > 0.0f ? vz : peakVelocityMps_;
        }
    }

    bool StandingJumpAnalyzer::HandleFlight(float az, uint64_t timestampUs, JumpMetrics& output)
    {
        if (timestampUs <= takeoffTimestampUs_) return false;

        const uint64_t flightDurationUs = timestampUs - takeoffTimestampUs_;

        if (flightDurationUs > MAX_FLIGHT_TIME_US)
        {
            resetCurrentJump();
            armed_ = false;
            return false;
        }

        if (flightDurationUs < MIN_FLIGHT_TIME_US) return false;

        if (az >= LANDING_IMPACT_ACCEL_MPS2)
        {
            const float flightTimeMs = static_cast<float>(flightDurationUs) / 1000.0f;
            const float flightTimeSeconds = static_cast<float>(flightDurationUs) / 1000000.0f;
            const float jumpHeightCm = 100.0f * 0.125f * GRAVITY_MPS2 * flightTimeSeconds * flightTimeSeconds;

            const uint32_t jumpIndex = jumpCount_;
            ++jumpCount_;
            jumpHeightSumCm_ += jumpHeightCm;

            JumpMetrics completed = lastCompletedJump_;
            completed.jump_number = jumpCount_;
            completed.takeoff_timestamp_us = takeoffTimestampUs_;
            completed.landing_timestamp_us = timestampUs;
            completed.flight_time_ms = flightTimeMs;
            completed.takeoff_velocity_mps = takeoffVelocityMps_;
            completed.peak_velocity_mps = peakVelocityMps_;
            completed.jump_height_cm = jumpHeightCm;

            if (jumpIndex < JumpMetrics::MAX_SET_JUMPS)
            {
                completed.jump_heights_cm[jumpIndex] = jumpHeightCm;
            }

            if (jumpHeightCm > completed.best_jump_height_cm)
            {
                completed.best_jump_height_cm = jumpHeightCm;
            }

            completed.mean_jump_height_cm = jumpHeightSumCm_ / static_cast<float>(jumpCount_);
            completed.valid = true;

            lastCompletedJump_ = completed;
            output = completed;

            resetCurrentJump();
            armed_ = false;
            return true;
        }
        return false;
    }
}
