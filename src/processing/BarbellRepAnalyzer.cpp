#include "processing/BarbellRepAnalyzer.h"

namespace vbt::processing
{
    void BarbellRepAnalyzer::onReset()
    {
        resetCurrentRep();
        repCount_ = 0;
        setMeanVelocitySum_ = 0.0f;
        lastCompletedRep_ = {};
    }

    void BarbellRepAnalyzer::resetCurrentRep()
    {
        phase_ = RepPhase::Idle;
        repStartTimestampUs_ = 0;
        concentricStartTimestampUs_ = 0;
        minPositionZ_ = 0.0f;
        maxPositionZ_ = 0.0f;
        concentricVelocitySum_ = 0.0f;
        concentricSampleCount_ = 0;
        peakVelocityMps_ = 0.0f;
        peakAccelMps2_ = 0.0f;
    }

    void BarbellRepAnalyzer::startConcentric(uint64_t timestampUs, float positionZ)
    {
        phase_ = RepPhase::Concentric;
        concentricStartTimestampUs_ = timestampUs;
        maxPositionZ_ = positionZ;
        concentricVelocitySum_ = 0.0f;
        concentricSampleCount_ = 0;
        peakVelocityMps_ = 0.0f;
    }

    bool BarbellRepAnalyzer::finalizeRep(uint64_t timestampUs, RepMetrics& output)
    {
        if (concentricSampleCount_ == 0 || timestampUs <= concentricStartTimestampUs_)
        {
            resetCurrentRep();
            return false;
        }

        const uint64_t concentricDurationUs = timestampUs - concentricStartTimestampUs_;
        const float rangeOfMotionM = maxPositionZ_ - minPositionZ_;

        if (concentricDurationUs < MIN_CONCENTRIC_DURATION_US || rangeOfMotionM < MIN_RANGE_OF_MOTION_M)
        {
            resetCurrentRep();
            return false;
        }

        const uint32_t repIndex = repCount_;
        ++repCount_;

        const float meanVelocity = concentricVelocitySum_ / static_cast<float>(concentricSampleCount_);
        setMeanVelocitySum_ += meanVelocity;

        RepMetrics completed = lastCompletedRep_;
        completed.rep_number = repCount_;
        completed.start_timestamp_us = repStartTimestampUs_;
        completed.concentric_start_us = concentricStartTimestampUs_;
        completed.concentric_end_us = timestampUs;
        completed.end_timestamp_us = timestampUs;
        completed.mean_concentric_velocity_mps = meanVelocity;
        completed.peak_concentric_velocity_mps = peakVelocityMps_;
        completed.peak_concentric_accel_mps2 = peakAccelMps2_;
        completed.range_of_motion_m = rangeOfMotionM;

        if (repIndex < RepMetrics::MAX_SET_REPS)
        {
            completed.rep_mean_velocity_mps[repIndex] = meanVelocity;
            completed.rep_peak_velocity_mps[repIndex] = peakVelocityMps_;
        }

        if (meanVelocity > completed.best_mean_velocity_mps)
        {
            completed.best_mean_velocity_mps = meanVelocity;
        }

        completed.set_mean_velocity_mps = setMeanVelocitySum_ / static_cast<float>(repCount_);

        if (completed.best_mean_velocity_mps > 0.001f)
        {
            const float drop = completed.best_mean_velocity_mps - meanVelocity;
            completed.velocity_loss_percent = (drop > 0.0f) ? (100.0f * drop / completed.best_mean_velocity_mps) : 0.0f;
        }

        completed.phase = RepPhase::Idle;
        completed.valid = true;

        lastCompletedRep_ = completed;
        output = completed;

        resetCurrentRep();
        return true;
    }

    bool BarbellRepAnalyzer::onFinishRecording(ActivityOutput& output)
    {
        // If user pressed STOP while finishing the top of a concentric drive, finalize it first.
        if (phase_ == RepPhase::Concentric)
        {
            finalizeRep(lastSampleTimestampUs(), output.rep);
        }
        else
        {
            resetCurrentRep();
        }

        if (repCount_ > 0)
        {
            output.rep = lastCompletedRep_;
            return true;
        }

        return false;
    }

    bool BarbellRepAnalyzer::onSample(const KinematicSample& sample,bool stationary,ActivityOutput& output)
    {
        const float vz = sample.velocity_mps.z;
        const float az = sample.linear_acceleration_mps2.z;
        const float pz = sample.position_m.z;

        switch (phase_)
        {
            case RepPhase::Idle:
                HandleIdle(vz, pz, sample);
                break;

            case RepPhase::Eccentric:
                HandleEccentric(pz, az, vz, sample, stationary);
                break;

            case RepPhase::Concentric:
                if (HandleConcentric(pz, az, vz, sample, stationary, output.rep))
                {
                    return true;
                }
                break;

            default:
                resetCurrentRep();
                break;
        }

        output.rep = lastCompletedRep_;
        output.rep.phase = phase_;
        return false;
    }

    void BarbellRepAnalyzer::HandleIdle(float vz, float pz, const KinematicSample& sample)
    {
        // A rep must begin with an eccentric descent.
        // Upward unracking motion from Idle is ignored.
        if (vz <= ECCENTRIC_START_VELOCITY_MPS)
        {
            phase_ = RepPhase::Eccentric;
            repStartTimestampUs_ = sample.timestamp_us;
            minPositionZ_ = pz;
            maxPositionZ_ = pz;
            peakAccelMps2_ = 0.0f;
        }
    }

    void BarbellRepAnalyzer::HandleEccentric(float pz, float az, float vz, const KinematicSample& sample, bool stationary)
    {
        if (pz < minPositionZ_) minPositionZ_ = pz;

        // Only retain continuous upward reversal acceleration at the bottom turnaround.
        // If acceleration is still downward (negative), reset the turnaround peak.
        if (az < 0.0f)
        {
            peakAccelMps2_ = 0.0f;
        }
        else if (az > peakAccelMps2_)
        {
            peakAccelMps2_ = az;
        }

        if (sample.timestamp_us - repStartTimestampUs_ > MAX_PHASE_DURATION_US)
        {
            resetCurrentRep();
            return;
        }

        const float eccentricDropM = maxPositionZ_ - minPositionZ_;

        if (vz >= CONCENTRIC_START_VELOCITY_MPS)
        {
            if (eccentricDropM >= MIN_RANGE_OF_MOTION_M)
            {
                startConcentric(sample.timestamp_us, pz);
            }
            else
            {
                resetCurrentRep();
            }
        }
        else if (stationary && eccentricDropM < MIN_RANGE_OF_MOTION_M)
        {
            resetCurrentRep();
        }
    }

    bool BarbellRepAnalyzer::HandleConcentric( float pz, float az, float vz, const KinematicSample& sample, bool stationary, RepMetrics& output)
    {
        if (sample.timestamp_us - concentricStartTimestampUs_ > MAX_PHASE_DURATION_US)
        {
            resetCurrentRep();
            return false;
        }

        if (vz > CONCENTRIC_END_VELOCITY_MPS && !stationary)
        {
            concentricVelocitySum_ += vz;
            ++concentricSampleCount_;

            if (vz > peakVelocityMps_) peakVelocityMps_ = vz;
            if (az > peakAccelMps2_) peakAccelMps2_ = az;
            if (pz > maxPositionZ_) maxPositionZ_ = pz;

            return false;
        }

        return finalizeRep(sample.timestamp_us, output);
    }
}
