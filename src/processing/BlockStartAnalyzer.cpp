#include "processing/BlockStartAnalyzer.h"

#include <cmath>

namespace vbt::processing
{
    float BlockStartAnalyzer::horizontalAccelerationMps2(const Vector3& linearAccel)
    {
        return std::sqrt(linearAccel.x * linearAccel.x + linearAccel.y * linearAccel.y);
    }

    float BlockStartAnalyzer::estimateDriveAngleDeg(const Vector3& linearAccel)
    {
        // Angle of total drive vector above the horizontal track (0 deg = horizontal, 90 deg = vertical)
        constexpr float RAD_TO_DEG = 57.2957795f;
        const float horiz = horizontalAccelerationMps2(linearAccel);
        const float vert = linearAccel.z > 0.0f ? linearAccel.z : 0.0f;
        return std::atan2(vert, horiz > 0.001f ? horiz : 0.001f) * RAD_TO_DEG;
    }

    void BlockStartAnalyzer::onReset()
    {
        resetCurrentStart();
        lastCompletedStart_ = {};
    }

    void BlockStartAnalyzer::resetCurrentStart()
    {
        phase_ = Phase::Idle;
        stepCount_ = 0;
        stationaryStartTimestampUs_ = 0;
        startTimestampUs_ = 0;
        stepTouchdownTimestampUs_ = 0;
        stepToeoffTimestampUs_ = 0;
        setPitchDeg_ = 0.0f;
        currentStepPeakAngleDeg_ = 0.0f;
        peakDriveAccelMps2_ = 0.0f;
        for (uint32_t i = 0; i < TARGET_BLOCK_STEPS; ++i)
        {
            stepContactTimesMs_[i] = 0.0f;
            stepDriveAnglesDeg_[i] = 0.0f;
        }
    }

    bool BlockStartAnalyzer::onFinishRecording(ActivityOutput& output)
    {
        if (stepCount_ > 0 || startTimestampUs_ != 0)
        {
            return finalizeBlockStart(output.block_start);
        }
        resetCurrentStart();
        if (lastCompletedStart_.valid)
        {
            output.block_start = lastCompletedStart_;
            return true;
        }
        return false;
    }

    bool BlockStartAnalyzer::onSample(const KinematicSample& sample, bool stationary, ActivityOutput& output)
    {
        switch (phase_)
        {
            case Phase::Idle:
                HandleIdle(sample.timestamp_us, stationary);
                break;
            case Phase::SetPosition:
                HandleSetPosition(sample, stationary);
                break;
            case Phase::FirstStepsStance:
                if (HandleFirstStepsStance(sample, stationary, output.block_start)) return true;
                break;
            case Phase::FirstStepsAerial:
                if (HandleFirstStepsAerial(sample, stationary, output.block_start)) return true;
                break;
            default:
                resetCurrentStart();
                break;
        }
        output.block_start = lastCompletedStart_;
        return false;
    }

    void BlockStartAnalyzer::HandleIdle(uint64_t timestampUs, bool stationary)
    {
        // One block start per recording session so walking back after the finish line
        // never overwrites the real start timestamp or first-4-step metrics.
        if (lastCompletedStart_.valid) return;

        if (!stationary)
        {
            stationaryStartTimestampUs_ = 0;
            return;
        }

        if (stationaryStartTimestampUs_ == 0)
        {
            stationaryStartTimestampUs_ = timestampUs;
        }

        if ((timestampUs - stationaryStartTimestampUs_) >= SET_HOLD_DURATION_US)
        {
            phase_ = Phase::SetPosition;
        }
    }

    void BlockStartAnalyzer::HandleSetPosition(const KinematicSample& sample, bool stationary)
    {
        if (stationary) return;

        const float totalDriveAccel = sample.linear_acceleration_mps2.magnitude();
        if (totalDriveAccel >= BLOCK_CLEARANCE_ACCEL_MPS2)
        {
            phase_ = Phase::FirstStepsStance;
            startTimestampUs_ = sample.timestamp_us;
            stepTouchdownTimestampUs_ = sample.timestamp_us;
            peakDriveAccelMps2_ = totalDriveAccel;
            currentStepPeakAngleDeg_ = estimateDriveAngleDeg(sample.linear_acceleration_mps2);
            setPitchDeg_ = currentStepPeakAngleDeg_;
        }
    }

    bool BlockStartAnalyzer::HandleFirstStepsStance(const KinematicSample& sample, bool stationary, BlockStartMetrics& output)
    {
        const uint64_t contactDurationUs = sample.timestamp_us - stepTouchdownTimestampUs_;
        const uint64_t maxContactUs = (stepCount_ == 0) ? MAX_BLOCK_PUSH_US : MAX_STEP_CONTACT_US;
        if (stationary || contactDurationUs > maxContactUs)
        {
            return finalizeBlockStart(output);
        }

        const float totalDriveAccel = sample.linear_acceleration_mps2.magnitude();
        if (totalDriveAccel > peakDriveAccelMps2_)
        {
            peakDriveAccelMps2_ = totalDriveAccel;
        }

        const float driveAngleDeg = estimateDriveAngleDeg(sample.linear_acceleration_mps2);
        if (driveAngleDeg > currentStepPeakAngleDeg_)
        {
            currentStepPeakAngleDeg_ = driveAngleDeg;
        }

        if (contactDurationUs >= MIN_STEP_CONTACT_US &&
            sample.linear_acceleration_mps2.z <= STEP_TOEOFF_ACCEL_MPS2)
        {
            if (stepCount_ < TARGET_BLOCK_STEPS)
            {
                stepContactTimesMs_[stepCount_] = static_cast<float>(contactDurationUs) / 1000.0f;
                stepDriveAnglesDeg_[stepCount_] = currentStepPeakAngleDeg_;
                ++stepCount_;
            }

            if (stepCount_ >= TARGET_BLOCK_STEPS)
            {
                return finalizeBlockStart(output);
            }

            phase_ = Phase::FirstStepsAerial;
            stepToeoffTimestampUs_ = sample.timestamp_us;
        }

        return false;
    }

    bool BlockStartAnalyzer::HandleFirstStepsAerial(const KinematicSample& sample, bool stationary, BlockStartMetrics& output)
    {
        const uint64_t aerialDurationUs = sample.timestamp_us - stepToeoffTimestampUs_;
        if (stationary || aerialDurationUs > MAX_STEP_AERIAL_US)
        {
            return finalizeBlockStart(output);
        }

        if (aerialDurationUs < MIN_STEP_AERIAL_US) return false;

        if (sample.linear_acceleration_mps2.z >= STEP_TOUCHDOWN_ACCEL_MPS2)
        {
            phase_ = Phase::FirstStepsStance;
            stepTouchdownTimestampUs_ = sample.timestamp_us;
            currentStepPeakAngleDeg_ = estimateDriveAngleDeg(sample.linear_acceleration_mps2);
        }

        return false;
    }

    bool BlockStartAnalyzer::finalizeBlockStart(BlockStartMetrics& output)
    {
        if (stepCount_ == 0 && startTimestampUs_ == 0)
        {
            resetCurrentStart();
            return false;
        }

        lastCompletedStart_ = {};
        lastCompletedStart_.step_count = stepCount_;
        lastCompletedStart_.start_timestamp_us = startTimestampUs_;
        for (uint32_t i = 0; i < stepCount_ && i < TARGET_BLOCK_STEPS; ++i)
        {
            lastCompletedStart_.step_contact_time_ms[i] = stepContactTimesMs_[i];
            lastCompletedStart_.step_pitch_deg[i] = stepDriveAnglesDeg_[i];
        }
        lastCompletedStart_.set_pitch_deg = setPitchDeg_;
        lastCompletedStart_.peak_drive_accel_mps2 = peakDriveAccelMps2_;
        lastCompletedStart_.valid = true;

        output = lastCompletedStart_;
        resetCurrentStart();
        return true;
    }
}
