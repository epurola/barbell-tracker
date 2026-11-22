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
        // TODO:
        // 1. Call resetCurrentStart()
        // 2. Reset lastCompletedStart_ = {}
    }

    void BlockStartAnalyzer::resetCurrentStart()
    {
        // TODO:
        // Reset phase_ = Phase::Idle, stepCount_ = 0,
        // timestamps to 0, setPitchDeg_ = 0.0f, currentStepPeakAngleDeg_ = 0.0f,
        // peakDriveAccelMps2_ = 0.0f, and zero out stepContactTimesMs_ & stepDriveAnglesDeg_
    }

    bool BlockStartAnalyzer::onFinishRecording(ActivityOutput& output)
    {
        if (stepCount_ > 0)
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
        // TODO:
        // Switch on phase_:
        // - Phase::Idle             -> HandleIdle(sample.timestamp_us, stationary)
        // - Phase::SetPosition      -> HandleSetPosition(sample, stationary)
        // - Phase::FirstStepsStance -> return HandleFirstStepsStance(sample, stationary, output.block_start)
        // - Phase::FirstStepsAerial -> return HandleFirstStepsAerial(sample, stationary, output.block_start)
        // - default                 -> resetCurrentStart()
        // Set output.block_start = lastCompletedStart_ and return false
        return false;
    }

    void BlockStartAnalyzer::HandleIdle(uint64_t timestampUs, bool stationary)
    {
        // TODO:
        // Wait for sprinter to hold still in the blocks ("Set" position):
        // 1. If !stationary: reset stationaryStartTimestampUs_ = 0 and return
        // 2. If stationaryStartTimestampUs_ == 0: set stationaryStartTimestampUs_ = timestampUs
        // 3. If (timestampUs - stationaryStartTimestampUs_) >= SET_HOLD_DURATION_US:
        //    phase_ = Phase::SetPosition
    }

    void BlockStartAnalyzer::HandleSetPosition(const KinematicSample& sample, bool stationary)
    {
        // TODO:
        // 1. If stationary, keep waiting for the gun / push-off
        // 2. Compute totalDriveAccel = sample.linear_acceleration_mps2.magnitude()
        // 3. When explosive block clearance starts (totalDriveAccel >= BLOCK_CLEARANCE_ACCEL_MPS2):
        //    - phase_ = Phase::FirstStepsStance
        //    - stepTouchdownTimestampUs_ = sample.timestamp_us
        //    - peakDriveAccelMps2_ = totalDriveAccel
        //    - currentStepPeakAngleDeg_ = estimateDriveAngleDeg(sample.linear_acceleration_mps2)
        //    - setPitchDeg_ = currentStepPeakAngleDeg_
    }

    bool BlockStartAnalyzer::HandleFirstStepsStance(const KinematicSample& sample, bool stationary, BlockStartMetrics& output)
    {
        // TODO:
        // 1. Compute contactDurationUs = sample.timestamp_us - stepTouchdownTimestampUs_
        // 2. If stationary or contactDurationUs > MAX_STEP_CONTACT_US:
        //    return finalizeBlockStart(output)
        // 3. Track peakDriveAccelMps2_ (using sample.linear_acceleration_mps2.magnitude())
        //    and currentStepPeakAngleDeg_ (using estimateDriveAngleDeg)
        // 4. Detect Toe-Off (contactDurationUs >= MIN_STEP_CONTACT_US &&
        //                    sample.linear_acceleration_mps2.z <= STEP_TOEOFF_ACCEL_MPS2):
        //    - Record stepContactTimesMs_[stepCount_] = contactDurationUs / 1000.0f
        //    - Record stepDriveAnglesDeg_[stepCount_] = currentStepPeakAngleDeg_
        //    - Increment ++stepCount_
        //    - If stepCount_ >= TARGET_BLOCK_STEPS (all 4 steps captured!):
        //        return finalizeBlockStart(output)
        //    - Otherwise transition to Phase::FirstStepsAerial and set stepToeoffTimestampUs_ = sample.timestamp_us
        return false;
    }

    bool BlockStartAnalyzer::HandleFirstStepsAerial(const KinematicSample& sample, bool stationary, BlockStartMetrics& output)
    {
        // TODO:
        // 1. Compute aerialDurationUs = sample.timestamp_us - stepToeoffTimestampUs_
        // 2. If stationary or aerialDurationUs > MAX_STEP_AERIAL_US:
        //    return finalizeBlockStart(output)
        // 3. If aerialDurationUs < MIN_STEP_AERIAL_US:
        //    return false
        // 4. Detect next step touchdown (sample.linear_acceleration_mps2.z >= STEP_TOUCHDOWN_ACCEL_MPS2):
        //    - phase_ = Phase::FirstStepsStance
        //    - stepTouchdownTimestampUs_ = sample.timestamp_us
        //    - currentStepPeakAngleDeg_ = estimateDriveAngleDeg(sample.linear_acceleration_mps2)
        return false;
    }

    bool BlockStartAnalyzer::finalizeBlockStart(BlockStartMetrics& output)
    {
        // TODO:
        // 1. If stepCount_ == 0:
        //    resetCurrentStart() and return false
        // 2. Fill output & lastCompletedStart_ (step_count, step_contact_time_ms[0..3],
        //    step_pitch_deg[0..3], set_pitch_deg, peak_drive_accel_mps2, valid = true)
        // 3. Call resetCurrentStart() and return true
        return false;
    }
}
