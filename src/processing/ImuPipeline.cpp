#include "processing/ImuPipeline.h"

namespace vbt::processing
{
    void ImuPipeline::reset()
    {
        expectedSequence_ = 0;
        havePreviousSequence_ = false;
        calibrator_.reset();
        filter_.reset();
        orientation_.reset();
        kinematics_.reset();
        counters_ = {};
    }

    bool ImuPipeline::validateAndTrackSequence(const RawImuSample& sample)
    {
        const bool valid = hasStatus(sample.status, SampleStatus::Valid) && hasStatus(sample.status, SampleStatus::AccelorometerOk) && hasStatus(sample.status, SampleStatus::GyroscopeOk) && hasStatus(sample.status, SampleStatus::TimestampValid) && hasStatus(sample.status, SampleStatus::SequenceValid);

        if (!valid)
        {
            ++counters_.invalidSamples;
            return false;
        }

        if (havePreviousSequence_ && sample.sequence != expectedSequence_)
        {
            counters_.sequenceGaps += (sample.sequence - expectedSequence_);
        }

        expectedSequence_ = sample.sequence + 1;
        havePreviousSequence_ = true;
        counters_.lastSequence = sample.sequence;
        ++counters_.processedSamples;

        return true;
    }

    bool ImuPipeline::process(const RawImuSample& sample, KinematicSample& output)
    {
        if (!validateAndTrackSequence(sample)) return false;

        using CalibratorState = ImuCalibrator::State;

        if (calibrator_.state() == CalibratorState::Collecting)
        {
            ++counters_.calibrationSamples;
            calibrator_.collect(sample);

            if (calibrator_.state() == CalibratorState::Failed)
            {
                ++counters_.calibrationFailures;
            }
            return false;
        }

        if (calibrator_.state() != CalibratorState::Ready) return false;

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

        if (!kinematics_.update(filtered, orientation, output))
        {
            ++counters_.kinematicFailures;
            return false;
        }
        ++counters_.kinematicSamples;

        return true;
    }
}
