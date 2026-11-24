#include "processing/SprintContactAnalyzer.h"

#include <algorithm>
#include <cmath>

namespace vbt::processing
{
    void SprintContactAnalyzer::onReset()
    {
        resetCurrentBout();
        lastCompletedSprint_ = {};
    }

    void SprintContactAnalyzer::resetCurrentBout()
    {
        phase_ = Phase::Idle;
        sawPropulsion_ = false;
        stepCount_ = {0};
        flightCount_ = {0};

        sprintStartTimestampUs_ = {0};
        lastStepEventTimestampUs_ = {0};
        touchdownTimestampUs_ = {0};
        toeoffTimestampUs_ = {0};

        lastContactTimeMs_ = {0.0f};
        contactTimeSumMs_ = {0.0f};
        minContactTimeMs_ = {0.0f};
        flightTimeSumMs_ = {0.0f};

        oddStepContactSumMs_ = {0.0f};
        oddStepCount_ = {0};
        evenStepContactSumMs_ = {0.0f};
        evenStepCount_ = {0};
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

    bool SprintContactAnalyzer::onSample(const KinematicSample& sample, bool stationary, ActivityOutput& output)
    {
        float az = sample.linear_acceleration_mps2.z;

        switch(phase_)
        {
            case Phase::Idle:   HandleIdle(az, sample.timestamp_us, stationary); break;
            case Phase::Stance: return HandleStance(az, sample.timestamp_us, stationary, output.sprint);
            case Phase::Aerial: return HandleAerial(az, sample.timestamp_us, stationary, output.sprint); 
            default: resetCurrentBout(); break;
        }

        output.sprint = lastCompletedSprint_;
        return false;
    }

    void SprintContactAnalyzer::HandleIdle(float az, uint64_t timestampUs, bool stationary)
    {
        if(!stationary && az >= TOUCHDOWN_IMPACT_ACCEL_MPS2)
        {
            phase_ = Phase::Stance;
            sprintStartTimestampUs_ = timestampUs;
            touchdownTimestampUs_ = timestampUs;
            lastStepEventTimestampUs_ = timestampUs;
            sawPropulsion_ = false;
        }
    }

    bool SprintContactAnalyzer::HandleStance(float az, uint64_t timestampUs, bool stationary, SprintContactMetrics& output)
    {
        uint64_t contactDuration = timestampUs - touchdownTimestampUs_;

        if(stationary || contactDuration > MAX_CONTACT_TIME_US) return finalizeSprintBout(timestampUs, output);

        if(az >= PROPULSION_ACCEL_MPS2) sawPropulsion_ = true;

        if(contactDuration >= MIN_CONTACT_TIME_US && sawPropulsion_ && az <= TOEOFF_FREEFALL_ACCEL_MPS2)
        {
            const float contactTimeMs = static_cast<float>(contactDuration) / 1000.0f;
            recordCompletedContact(contactTimeMs);
            phase_ = Phase::Aerial;
            toeoffTimestampUs_ = timestampUs;
            lastStepEventTimestampUs_ = timestampUs;
        }
        
        return false; 
    }

    bool SprintContactAnalyzer::HandleAerial(float az, uint64_t timestampUs, bool stationary, SprintContactMetrics& output)
    {

        uint64_t flightDuration = timestampUs - toeoffTimestampUs_;

        if(stationary || flightDuration > MAX_FLIGHT_TIME_US || timestampUs - lastStepEventTimestampUs_ > SPRINT_END_TIMEOUT_US ) return finalizeSprintBout(timestampUs, output);
        
        if(flightDuration < MIN_FLIGHT_TIME_US) return false;

        if(az >= TOUCHDOWN_IMPACT_ACCEL_MPS2)
        {
            const float flightTimeMs = static_cast<float>(flightDuration) / 1000.0f;
            flightTimeSumMs_ += flightTimeMs;
            ++flightCount_;
            phase_ = Phase::Stance;
            touchdownTimestampUs_ = timestampUs;
            lastStepEventTimestampUs_ = timestampUs;
            sawPropulsion_ = false;
        }

        return false;
    }

    void SprintContactAnalyzer::recordCompletedContact(float contactTimeMs)
    {
        if (stepCount_ < MAX_TRACKED_STEPS) stepContactTimesMs_[stepCount_] = contactTimeMs;
        ++stepCount_;
        lastContactTimeMs_ = contactTimeMs;
        contactTimeSumMs_ += contactTimeMs;
        if(stepCount_ == 1) minContactTimeMs_ = contactTimeMs;
        minContactTimeMs_ = std::min(contactTimeMs, minContactTimeMs_);

        if(stepCount_ % 2 == 1)
        {
            oddStepContactSumMs_ += contactTimeMs;
            oddStepCount_++;
        }
        else
        {
            evenStepContactSumMs_ += contactTimeMs;
            ++evenStepCount_;
        }
    }

    bool SprintContactAnalyzer::finalizeSprintBout(uint64_t timestampUs, SprintContactMetrics& output)
    {
        if(stepCount_ < MIN_VALID_SPRINT_STEPS) 
        {
            resetCurrentBout();
            return false;
        }

        output = {};
        output.step_count = stepCount_;
        output.sprint_start_timestamp_us = sprintStartTimestampUs_;
        output.sprint_end_timestamp_us = timestampUs;
        output.last_contact_time_ms = lastContactTimeMs_;
        output.min_contact_time_ms = minContactTimeMs_;
        output.mean_contact_time_ms = contactTimeSumMs_ / static_cast<float>(stepCount_);
        output.mean_flight_time_ms = (flightCount_ > 0) ? (flightTimeSumMs_ / static_cast<float>(flightCount_)) : 0.0f;

        const float stepPeriodSeconds = (output.mean_contact_time_ms + output.mean_flight_time_ms) / 1000.0f;
        output.step_frequency_hz = (stepPeriodSeconds > 0.0f) ? (1.0f / stepPeriodSeconds) : 0.0f;

        if(oddStepCount_ > 0 && evenStepCount_ > 0 && output.mean_contact_time_ms > 0.0f)
        {
            const float oddMean = oddStepContactSumMs_ / static_cast<float>(oddStepCount_);
            const float evenMean = evenStepContactSumMs_ / static_cast<float>(evenStepCount_);
            output.odd_even_asymmetry_percent = 100.0f * std::fabs(oddMean - evenMean) / output.mean_contact_time_ms;
        }

        const uint32_t trackedSteps = std::min(stepCount_, MAX_TRACKED_STEPS);
        const uint32_t half = trackedSteps / 2;
        if(half > 0)
        {
            float firstHalfSumMs = 0.0f;
            float secondHalfSumMs = 0.0f;
            for(uint32_t i = 0; i < half; ++i)
            {
                firstHalfSumMs += stepContactTimesMs_[i];
                secondHalfSumMs += stepContactTimesMs_[trackedSteps - half + i];
            }
            output.contact_drift_ms = (secondHalfSumMs - firstHalfSumMs) / static_cast<float>(half);
        }

        output.valid = true;
        lastCompletedSprint_ = output;
        resetCurrentBout();
        return true;
    }
}
