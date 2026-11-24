#include "processing/HurdleContactAnalyzer.h"

namespace vbt::processing
{
    void HurdleContactAnalyzer::onReset()
    {
        resetCurrentHop();
        contactCount_ = 0;
        minContactTimeMs_ = 0.0f;
        contactTimeSumMs_ = 0.0f;
        rsiSum_ = 0.0f;
        lastCompletedContact_ = {};
    }

    void HurdleContactAnalyzer::resetCurrentHop()
    {
        phase_ = Phase::Idle;
        sawReboundPush_ = false;
        approachTakeoffTimestampUs_ = 0;
        touchdownTimestampUs_ = 0;
        toeoffTimestampUs_ = 0;
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
        float az = sample.linear_acceleration_mps2.z;

        switch(phase_)
        {
            case Phase::Idle:           HandleIdle(az, sample.timestamp_us); break;
            case Phase::ApproachFlight: HandleApproachFlight(az, sample.timestamp_us, stationary); break;
            case Phase::GroundContact:  HandleGroundContact(az, sample.timestamp_us, stationary); break;
            case Phase::ReboundFlight:
                if (HandleReboundFlight(az, sample.timestamp_us, output.hurdle)) return true;
                break;
            default: resetCurrentHop();
        }
        output.hurdle = lastCompletedContact_;
        return false;
    }

    void HurdleContactAnalyzer::HandleIdle(float az, uint64_t timestampUs)
    {
        if(az <= FREEFALL_ACCEL_MPS2)
        {
            phase_ = Phase::ApproachFlight;
            approachTakeoffTimestampUs_ = timestampUs;
        }
    }

    void HurdleContactAnalyzer::HandleApproachFlight(float az, uint64_t timestampUs, bool stationary)
    {
        if(stationary || timestampUs - approachTakeoffTimestampUs_ > MAX_REBOUND_FLIGHT_US)
        {
            resetCurrentHop(); 
            return;
        }

        if(timestampUs -approachTakeoffTimestampUs_ <  MIN_APPROACH_FLIGHT_US) return;

        if(az >= TOUCHDOWN_IMPACT_ACCEL_MPS2)
        {
            phase_ = Phase::GroundContact;
            touchdownTimestampUs_ = timestampUs;
            sawReboundPush_ = false;
        }
    }

    void HurdleContactAnalyzer::HandleGroundContact(float az, uint64_t timestampUs, bool stationary)
    {
        uint64_t contactDuration = timestampUs -touchdownTimestampUs_;
        
        if(stationary || contactDuration > MAX_CONTACT_TIME_US)
        {
            resetCurrentHop();
            return;
        }

        if(az >= REBOUND_PUSH_ACCEL_MPS2) sawReboundPush_ = true;

        if(contactDuration >= MIN_CONTACT_TIME_US && sawReboundPush_ && az <= FREEFALL_ACCEL_MPS2)
        {
            phase_ = Phase::ReboundFlight;
            toeoffTimestampUs_ = timestampUs;
        }
    }

    bool HurdleContactAnalyzer::HandleReboundFlight(float az, uint64_t timestampUs, HurdleContactMetrics& output)
    {
        const uint64_t flightDurationUs = timestampUs - toeoffTimestampUs_;

        if (flightDurationUs > MAX_REBOUND_FLIGHT_US)
        {
            resetCurrentHop();
            return false;
        }

        if (flightDurationUs < MIN_REBOUND_FLIGHT_US) return false;
        if (az < TOUCHDOWN_IMPACT_ACCEL_MPS2) return false;

        const uint64_t contactDurationUs = (toeoffTimestampUs_ > touchdownTimestampUs_)
            ? (toeoffTimestampUs_ - touchdownTimestampUs_)
            : 0;
        if (contactDurationUs == 0)
        {
            resetCurrentHop();
            return false;
        }

        const float contactTimeMs = static_cast<float>(contactDurationUs) / 1000.0f;
        const float contactTimeSeconds = static_cast<float>(contactDurationUs) / 1000000.0f;
        const float flightTimeMs = static_cast<float>(flightDurationUs) / 1000.0f;
        const float flightTimeSeconds = static_cast<float>(flightDurationUs) / 1000000.0f;
        const float jumpHeightMeters = 0.125f * GRAVITY_MPS2 * flightTimeSeconds * flightTimeSeconds;
        const float jumpHeightCm = 100.0f * jumpHeightMeters;
        const float rsi = jumpHeightMeters / contactTimeSeconds;

        const uint32_t hopIndex = contactCount_;
        ++contactCount_;

        if (contactCount_ == 1 || contactTimeMs < minContactTimeMs_)
        {
            minContactTimeMs_ = contactTimeMs;
        }
        contactTimeSumMs_ += contactTimeMs;
        rsiSum_ += rsi;

        if (hopIndex < HurdleContactMetrics::MAX_SET_HURDLES)
        {
            lastCompletedContact_.hop_contact_time_ms[hopIndex] = contactTimeMs;
            lastCompletedContact_.hop_rsi[hopIndex] = rsi;
        }

        lastCompletedContact_.contact_number = contactCount_;
        lastCompletedContact_.touchdown_timestamp_us = touchdownTimestampUs_;
        lastCompletedContact_.toeoff_timestamp_us = toeoffTimestampUs_;
        lastCompletedContact_.next_landing_timestamp_us = timestampUs;
        lastCompletedContact_.contact_time_ms = contactTimeMs;
        lastCompletedContact_.min_contact_time_ms = minContactTimeMs_;
        lastCompletedContact_.mean_contact_time_ms = contactTimeSumMs_ / static_cast<float>(contactCount_);
        lastCompletedContact_.flight_time_ms = flightTimeMs;
        lastCompletedContact_.jump_height_cm = jumpHeightCm;
        lastCompletedContact_.reactive_strength_index = rsi;
        if (contactCount_ == 1 || rsi > lastCompletedContact_.best_rsi)
        {
            lastCompletedContact_.best_rsi = rsi;
        }
        lastCompletedContact_.mean_rsi = rsiSum_ / static_cast<float>(contactCount_);
        lastCompletedContact_.valid = true;

        output = lastCompletedContact_;

        // Chain directly into the next hurdle contact.
        phase_ = Phase::GroundContact;
        touchdownTimestampUs_ = timestampUs;
        toeoffTimestampUs_ = 0;
        sawReboundPush_ = false;
        return true;
    }
}
