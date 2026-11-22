#include "processing/ImuPipeline.h"

namespace vbt::processing
{
    ImuPipeline::ImuPipeline()
        : sessionState_(SessionState::Calibrating),
          setNumber_(1),
          activeAnalyzer_(&barbellAnalyzer_)
    {
    }

    ActivityAnalyzer* ImuPipeline::analyzerForMode(ActivityMode mode)
    {
        switch (mode)
        {
            case ActivityMode::BarbellVbt:    return &barbellAnalyzer_;
            case ActivityMode::StandingJump:  return &standingJumpAnalyzer_;
            case ActivityMode::HurdleContact: return &hurdleAnalyzer_;
            case ActivityMode::SprintContact: return &sprintAnalyzer_;
            case ActivityMode::BlockStart:    return &blockStartAnalyzer_;
            default:                          return &barbellAnalyzer_;
        }
    }

    void ImuPipeline::setMode(ActivityMode mode)
    {
        startSession(mode, true);
    }

    void ImuPipeline::startSession(ActivityMode mode, bool recalibrate)
    {
        ActivityAnalyzer* target = analyzerForMode(mode);
        const bool modeChanged = (target != activeAnalyzer_);

        if (modeChanged)
        {
            activeAnalyzer_->reset();
            activeAnalyzer_ = target;
            setNumber_ = 1;
        }
        else if (!recalibrate)
        {
            ++setNumber_;
        }
        else
        {
            setNumber_ = 1;
        }

        activeAnalyzer_->reset();
        kinematics_.reset();

        // Use 20 Hz low-pass for Barbell VBT & Standing Jump, and 50 Hz for high-speed ground contacts.
        const float cutoffHz =
            (mode == ActivityMode::BarbellVbt || mode == ActivityMode::StandingJump)
                ? ImuFilter::CUTOFF_HZ
                : ImuFilter::HIGH_SPEED_CUTOFF_HZ;
        filter_.setCutoffHz(cutoffHz);

        lastCompletedActivity_ = {};
        lastCompletedActivity_.mode = mode;
        lastCompletedActivity_.set_number = setNumber_;

        if (recalibrate || calibrator_.state() != ImuCalibrator::State::Ready)
        {
            calibrator_.reset();
            filter_.reset();
            orientation_.reset();
            sessionState_ = SessionState::Calibrating;
        }
        else
        {
            kinematics_.setGravityMagnitude(calibrator_.calibratedGravityMagnitudeMps2());
            sessionState_ = SessionState::Recording;
        }

        lastCompletedActivity_.session_state = sessionState_;
    }

    void ImuPipeline::startNextSet()
    {
        startSession(activeAnalyzer_->mode(), false);
    }

    void ImuPipeline::stopSession(ActivityOutput& finalOutput)
    {
        if (sessionState_ == SessionState::Recording)
        {
            ActivityOutput flushed = lastCompletedActivity_;
            if (activeAnalyzer_->finishRecording(flushed))
            {
                flushed.set_number = setNumber_;
                lastCompletedActivity_ = flushed;
            }
        }

        sessionState_ = SessionState::ResultsReady;
        lastCompletedActivity_.session_state = sessionState_;
        lastCompletedActivity_.effort_in_progress = false;
        finalOutput = lastCompletedActivity_;
    }

    void ImuPipeline::returnToModeSelect()
    {
        activeAnalyzer_->reset();
        kinematics_.reset();
        setNumber_ = 0;
        sessionState_ = SessionState::IdleModeSelect;
        lastCompletedActivity_.session_state = sessionState_;
        lastCompletedActivity_.effort_in_progress = false;
    }

    void ImuPipeline::reset()
    {
        expectedSequence_ = 0;
        havePreviousSequence_ = false;
        calibrator_.reset();
        filter_.reset();
        orientation_.reset();
        kinematics_.reset();
        barbellAnalyzer_.reset();
        standingJumpAnalyzer_.reset();
        hurdleAnalyzer_.reset();
        sprintAnalyzer_.reset();
        blockStartAnalyzer_.reset();
        setNumber_ = 1;
        sessionState_ = SessionState::Calibrating;
        lastCompletedActivity_ = {};
        counters_ = {};
    }

    bool ImuPipeline::validateAndTrackSequence(const RawImuSample& sample)
    {
        const bool valid =
            hasStatus(sample.status, SampleStatus::Valid) &&
            hasStatus(sample.status, SampleStatus::AccelorometerOk) &&
            hasStatus(sample.status, SampleStatus::GyroscopeOk) &&
            hasStatus(sample.status, SampleStatus::TimestampValid) &&
            hasStatus(sample.status, SampleStatus::SequenceValid);

        if (!valid)
        {
            ++counters_.invalidSamples;
            return false;
        }

        if (havePreviousSequence_ && sample.sequence > expectedSequence_)
        {
            counters_.sequenceGaps += (sample.sequence - expectedSequence_);
        }

        expectedSequence_ = sample.sequence + 1;
        havePreviousSequence_ = true;
        counters_.lastSequence = sample.sequence;
        ++counters_.processedSamples;

        return true;
    }

    bool ImuPipeline::process(const RawImuSample& sample, KinematicSample& kinematicsOut, ActivityOutput& activityOut)
    {
        activityOut.mode = activeAnalyzer_->mode();
        activityOut.session_state = sessionState_;
        activityOut.set_number = setNumber_;
        activityOut.completed = false;
        activityOut.effort_in_progress = activeAnalyzer_->isEffortInProgress();

        if (!validateAndTrackSequence(sample)) return false;

        // When browsing mode cards or viewing completed set results, pause pipeline analysis.
        if (sessionState_ == SessionState::IdleModeSelect ||
            sessionState_ == SessionState::ResultsReady)
        {
            return false;
        }

        using CalibratorState = ImuCalibrator::State;

        if (calibrator_.state() == CalibratorState::Collecting)
        {
            sessionState_ = SessionState::Calibrating;
            activityOut.session_state = sessionState_;

            ++counters_.calibrationSamples;
            calibrator_.collect(sample);

            if (calibrator_.state() == CalibratorState::Failed)
            {
                ++counters_.calibrationFailures;
                calibrator_.reset();
            }
            else if (calibrator_.state() == CalibratorState::Ready)
            {
                kinematics_.setGravityMagnitude(calibrator_.calibratedGravityMagnitudeMps2());
                sessionState_ = SessionState::Recording;
                activityOut.session_state = sessionState_;
            }
            return false;
        }

        if (calibrator_.state() != CalibratorState::Ready) return false;

        sessionState_ = SessionState::Recording;
        activityOut.session_state = sessionState_;

        CalibratedImuSample calibrated{};
        if (!calibrator_.apply(sample, calibrated)) return false;
        ++counters_.calibratedSamples;

        FilteredImuSample filtered{};
        if (!filter_.process(calibrated, filtered))
        {
            ++counters_.filterFailures;
            return false;
        }
        ++counters_.filteredSamples;

        OrientationState orientation{};
        if (!orientation_.update(filtered, orientation))
        {
            ++counters_.orientationFailures;
            return false;
        }
        ++counters_.orientationSamples;

        if (!kinematics_.update(filtered, orientation, kinematicsOut))
        {
            ++counters_.kinematicFailures;
            return false;
        }
        ++counters_.kinematicSamples;

        if (activeAnalyzer_->update(kinematicsOut, activityOut))
        {
            ++counters_.completedEfforts;
            activityOut.session_state = sessionState_;
            activityOut.set_number = setNumber_;
            lastCompletedActivity_ = activityOut;
        }

        return true;
    }
}
