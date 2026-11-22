#include "processing/HurdleContactAnalyzer.h"

namespace vbt::processing
{
    void HurdleContactAnalyzer::onReset()
    {
        // TODO:
        // 1. Call resetCurrentHop()
        // 2. Reset contactCount_ = 0, minContactTimeMs_ = 0.0f, contactTimeSumMs_ = 0.0f,
        //    rsiSum_ = 0.0f, and lastCompletedContact_ = {}
    }

    void HurdleContactAnalyzer::resetCurrentHop()
    {
        // TODO:
        // Reset phase_ = Phase::Idle, sawReboundPush_ = false,
        // approachTakeoffTimestampUs_ = 0, touchdownTimestampUs_ = 0, toeoffTimestampUs_ = 0
    }

    bool HurdleContactAnalyzer::onFinishRecording(ActivityOutput& output)
    {
        resetCurrentHop();
        if (contactCount_ > 0)
        {
            output.hurdle = lastCompletedContact_;
            return true;
        }
        return false;
    }

    bool HurdleContactAnalyzer::onSample(const KinematicSample& sample, bool stationary, ActivityOutput& output)
    {
        // TODO:
        // 1. Read az = sample.linear_acceleration_mps2.z
        // 2. Switch on phase_:
        //    - Phase::Idle           -> HandleIdle(az, sample.timestamp_us)
        //    - Phase::ApproachFlight -> HandleApproachFlight(az, sample.timestamp_us, stationary)
        //    - Phase::GroundContact  -> HandleGroundContact(az, sample.timestamp_us, stationary)
        //    - Phase::ReboundFlight  -> return HandleReboundFlight(az, sample.timestamp_us, output.hurdle)
        //    - default               -> resetCurrentHop()
        // 3. Set output.hurdle = lastCompletedContact_ and return false
        return false;
    }

    void HurdleContactAnalyzer::HandleIdle(float az, uint64_t timestampUs)
    {
        // TODO:
        // When athlete jumps over the first hurdle (or drops off a box), they enter free-fall:
        // If az <= FREEFALL_ACCEL_MPS2:
        //   phase_ = Phase::ApproachFlight
        //   approachTakeoffTimestampUs_ = timestampUs
    }

    void HurdleContactAnalyzer::HandleApproachFlight(float az, uint64_t timestampUs, bool stationary)
    {
        // TODO:
        // 1. If stationary or (timestampUs - approachTakeoffTimestampUs_ > MAX_REBOUND_FLIGHT_US):
        //    resetCurrentHop() and return
        // 2. If (timestampUs - approachTakeoffTimestampUs_ < MIN_APPROACH_FLIGHT_US):
        //    return (still early in air)
        // 3. Detect landing impact between hurdles:
        //    If az >= TOUCHDOWN_IMPACT_ACCEL_MPS2:
        //      phase_ = Phase::GroundContact
        //      touchdownTimestampUs_ = timestampUs
        //      sawReboundPush_ = false
    }

    void HurdleContactAnalyzer::HandleGroundContact(float az, uint64_t timestampUs, bool stationary)
    {
        // TODO:
        // 1. Compute contactDurationUs = timestampUs - touchdownTimestampUs_
        // 2. If stationary or contactDurationUs > MAX_CONTACT_TIME_US:
        //    (Athlete stuck the landing and stopped instead of rebounding over another hurdle)
        //    resetCurrentHop() and return
        // 3. If az >= REBOUND_PUSH_ACCEL_MPS2:
        //    sawReboundPush_ = true
        // 4. Detect Toe-Off into the rebound jump:
        //    If contactDurationUs >= MIN_CONTACT_TIME_US && sawReboundPush_ && az <= FREEFALL_ACCEL_MPS2:
        //      phase_ = Phase::ReboundFlight
        //      toeoffTimestampUs_ = timestampUs
    }

    bool HurdleContactAnalyzer::HandleReboundFlight(float az, uint64_t timestampUs, HurdleContactMetrics& output)
    {
        // TODO:
        // 1. Compute flightDurationUs = timestampUs - toeoffTimestampUs_
        // 2. If flightDurationUs > MAX_REBOUND_FLIGHT_US:
        //    resetCurrentHop() and return false
        // 3. If flightDurationUs < MIN_REBOUND_FLIGHT_US:
        //    return false
        // 4. When next landing impact happens (az >= TOUCHDOWN_IMPACT_ACCEL_MPS2):
        //    - Compute contactDurationUs = toeoffTimestampUs_ - touchdownTimestampUs_
        //    - contactTimeMs = contactDurationUs / 1000.0f
        //    - contactTimeSeconds = contactDurationUs / 1000000.0f
        //    - flightTimeMs = flightDurationUs / 1000.0f
        //    - flightTimeSeconds = flightDurationUs / 1000000.0f
        //    - jumpHeightMeters = 0.125f * GRAVITY_MPS2 * flightTimeSeconds * flightTimeSeconds
        //    - jumpHeightCm = 100.0f * jumpHeightMeters
        //    - rsi = jumpHeightMeters / contactTimeSeconds
        //    - Store in hop_contact_time_ms[hopIndex] and hop_rsi[hopIndex] (up to MAX_SET_HURDLES)
        //    - Update minContactTimeMs_, mean_contact_time_ms, best_rsi, and mean_rsi
        //    - Chain directly into the next hurdle contact:
        //        phase_ = Phase::GroundContact;
        //        touchdownTimestampUs_ = timestampUs;
        //        toeoffTimestampUs_ = 0;
        //        sawReboundPush_ = false;
        //    - return true
        return false;
    }
}
