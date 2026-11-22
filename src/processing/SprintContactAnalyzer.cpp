#include "processing/SprintContactAnalyzer.h"

#include <cmath>

namespace vbt::processing
{
    void SprintContactAnalyzer::onReset()
    {
        // TODO:
        // 1. Call resetCurrentBout()
        // 2. Reset lastCompletedSprint_ = {}
    }

    void SprintContactAnalyzer::resetCurrentBout()
    {
        // TODO:
        // Reset phase_ = Phase::Idle, sawPropulsion_ = false,
        // stepCount_ = 0, flightCount_ = 0,
        // all timestamps to 0, all sums/min/last floats to 0.0f,
        // oddStepCount_ = 0, evenStepCount_ = 0
    }

    bool SprintContactAnalyzer::onFinishRecording(ActivityOutput& output)
    {
        if (stepCount_ >= MIN_VALID_SPRINT_STEPS)
        {
            return finalizeSprintBout(lastSampleTimestampUs(), output.sprint);
        }
        resetCurrentBout();
        if (lastCompletedSprint_.valid)
        {
            output.sprint = lastCompletedSprint_;
            return true;
        }
        return false;
    }

    bool SprintContactAnalyzer::onSample(
        const KinematicSample& sample,
        bool stationary,
        ActivityOutput& output)
    {
        // TODO:
        // 1. Read az = sample.linear_acceleration_mps2.z
        // 2. Switch on phase_:
        //    - Phase::Idle   -> HandleIdle(az, sample.timestamp_us, stationary)
        //    - Phase::Stance -> return HandleStance(az, sample.timestamp_us, stationary, output.sprint)
        //    - Phase::Aerial -> return HandleAerial(az, sample.timestamp_us, stationary, output.sprint)
        //    - default       -> resetCurrentBout()
        // 3. Set output.sprint = lastCompletedSprint_ and return false
        return false;
    }

    void SprintContactAnalyzer::HandleIdle(float az, uint64_t timestampUs, bool stationary)
    {
        // TODO:
        // If not stationary and foot strike impact occurs (az >= TOUCHDOWN_IMPACT_ACCEL_MPS2):
        //   phase_ = Phase::Stance
        //   sprintStartTimestampUs_ = timestampUs
        //   touchdownTimestampUs_ = timestampUs
        //   lastStepEventTimestampUs_ = timestampUs
        //   sawPropulsion_ = false
    }

    bool SprintContactAnalyzer::HandleStance(float az, uint64_t timestampUs, bool stationary, SprintContactMetrics& output)
    {
        // TODO:
        // 1. Compute contactDurationUs = timestampUs - touchdownTimestampUs_
        // 2. If stationary or contactDurationUs > MAX_CONTACT_TIME_US:
        //    (Sprinter stopped on the ground; finalize the sprint bout!)
        //    return finalizeSprintBout(timestampUs, output)
        // 3. If az >= PROPULSION_ACCEL_MPS2:
        //    sawPropulsion_ = true
        // 4. Detect Toe-Off into Aerial phase:
        //    If contactDurationUs >= MIN_CONTACT_TIME_US && sawPropulsion_ && az <= TOEOFF_FREEFALL_ACCEL_MPS2:
        //      const float contactTimeMs = static_cast<float>(contactDurationUs) / 1000.0f;
        //      recordCompletedContact(contactTimeMs);
        //      phase_ = Phase::Aerial;
        //      toeoffTimestampUs_ = timestampUs;
        //      lastStepEventTimestampUs_ = timestampUs;
        return false;
    }

    bool SprintContactAnalyzer::HandleAerial(float az, uint64_t timestampUs, bool stationary, SprintContactMetrics& output)
    {
        // TODO:
        // 1. Compute flightDurationUs = timestampUs - toeoffTimestampUs_
        // 2. If stationary or flightDurationUs > MAX_FLIGHT_TIME_US or
        //    (timestampUs - lastStepEventTimestampUs_ > SPRINT_END_TIMEOUT_US):
        //    (Sprinter finished the wicket run / sprint bout!)
        //    return finalizeSprintBout(timestampUs, output)
        // 3. If flightDurationUs < MIN_FLIGHT_TIME_US:
        //    return false (still early in aerial phase between steps)
        // 4. Detect next foot strike (az >= TOUCHDOWN_IMPACT_ACCEL_MPS2):
        //    const float flightTimeMs = static_cast<float>(flightDurationUs) / 1000.0f;
        //    flightTimeSumMs_ += flightTimeMs;
        //    ++flightCount_;
        //    phase_ = Phase::Stance;
        //    touchdownTimestampUs_ = timestampUs;
        //    lastStepEventTimestampUs_ = timestampUs;
        //    sawPropulsion_ = false;
        return false;
    }

    void SprintContactAnalyzer::recordCompletedContact(float contactTimeMs)
    {
        // TODO:
        // 1. Store in stepContactTimesMs_[stepCount_] if stepCount_ < MAX_TRACKED_STEPS
        // 2. Increment ++stepCount_
        // 3. Update lastContactTimeMs_, contactTimeSumMs_, and minContactTimeMs_
        // 4. If (stepCount_ % 2 == 1) add to oddStepContactSumMs_ and ++oddStepCount_
        //    else add to evenStepContactSumMs_ and ++evenStepCount_
    }

    bool SprintContactAnalyzer::finalizeSprintBout(uint64_t timestampUs, SprintContactMetrics& output)
    {
        // TODO:
        // 1. If stepCount_ < MIN_VALID_SPRINT_STEPS:
        //    resetCurrentBout() and return false
        // 2. Compute:
        //    - meanContactMs = contactTimeSumMs_ / stepCount_
        //    - meanFlightMs = (flightCount_ > 0) ? (flightTimeSumMs_ / flightCount_) : 0.0f
        //    - stepPeriodSeconds = (meanContactMs + meanFlightMs) / 1000.0f
        //    - stepFrequencyHz = (stepPeriodSeconds > 0.0f) ? (1.0f / stepPeriodSeconds) : 0.0f
        //    - Odd vs. Even Asymmetry %:
        //      oddMean = oddStepContactSumMs_ / oddStepCount_
        //      evenMean = evenStepContactSumMs_ / evenStepCount_
        //      asymmetryPercent = 100.0f * std::fabs(oddMean - evenMean) / meanContactMs
        //    - First-half vs. Second-half Contact Drift (ms):
        //      Average the first (trackedSteps / 2) steps vs. the last (trackedSteps / 2) steps in stepContactTimesMs_
        //      contactDriftMs = secondHalfMeanMs - firstHalfMeanMs
        // 3. Fill output & lastCompletedSprint_, call resetCurrentBout(), and return true
        return false;
    }
}
