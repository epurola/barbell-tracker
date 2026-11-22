#pragma once

#include <cmath>
#include <cstdint>

#include "contracts/FirmwareContracts.h"

namespace vbt::processing
{
    class ActivityAnalyzer
    {
    public:
        virtual ~ActivityAnalyzer() = default;

        virtual ActivityMode mode() const = 0;
        virtual bool isEffortInProgress() const = 0;

        void reset()
        {
            previousTimestampUs_ = 0;
            onReset();
        }

        // Common entry point: validates the sample and timestamp once,
        // then delegates to the derived analyzer's onSample().
        bool update(const KinematicSample& sample, ActivityOutput& output)
        {
            output.mode = mode();
            output.completed = false;

            if (!validateAndAdvanceTimestamp(sample))
            {
                output.effort_in_progress = isEffortInProgress();
                return false;
            }

            const bool stationary = hasStatus(sample.status, SampleStatus::Stationary);
            const bool completed = onSample(sample, stationary, output);

            output.mode = mode();
            output.effort_in_progress = isEffortInProgress();
            output.completed = completed;

            return completed;
        }

        // Called when the user manually presses STOP to end the set/run recording window.
        // Finalizes any in-progress effort and populates `output` with the full set summary.
        bool finishRecording(ActivityOutput& output)
        {
            output.mode = mode();
            const bool hasResult = onFinishRecording(output);
            output.effort_in_progress = false;
            output.completed = hasResult;
            return hasResult;
        }

    protected:
        virtual void onReset() = 0;
        virtual bool onSample(const KinematicSample& sample, bool stationary, ActivityOutput& output) = 0;

        virtual bool onFinishRecording(ActivityOutput& /*output*/)
        {
            return false;
        }

        uint64_t lastSampleTimestampUs() const { return previousTimestampUs_; }

    private:
        bool validateAndAdvanceTimestamp(const KinematicSample& sample)
        {
            if (!hasStatus(sample.status, SampleStatus::KinematicsValid)) return false;

            if (!std::isfinite(sample.velocity_mps.z) ||
                !std::isfinite(sample.linear_acceleration_mps2.z) ||
                !std::isfinite(sample.position_m.z))
            {
                return false;
            }

            if (previousTimestampUs_ != 0 && sample.timestamp_us <= previousTimestampUs_)
            {
                return false;
            }

            previousTimestampUs_ = sample.timestamp_us;
            return true;
        }

        uint64_t previousTimestampUs_{0};
    };
}
