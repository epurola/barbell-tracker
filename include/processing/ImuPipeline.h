#pragma once

#include <cstdint>

#include "processing/ImuCalibration.h"
#include "processing/ImuFilter.h"
#include "processing/OrientationEstimator.h"
#include "processing/kinematicsEstimator.h"

namespace vbt::processing
{
    struct PipelineCounters
    {
        uint32_t processedSamples{0};
        uint32_t invalidSamples{0};
        uint32_t sequenceGaps{0};
        uint32_t lastSequence{0};
        uint32_t calibrationSamples{0};
        uint32_t calibratedSamples{0};
        uint32_t calibrationFailures{0};
        uint32_t filteredSamples{0};
        uint32_t filterFailures{0};
        uint32_t orientationSamples{0};
        uint32_t orientationFailures{0};
        uint32_t kinematicSamples{0};
        uint32_t kinematicFailures{0};
    };

    class ImuPipeline
    {
    public:
        // Processes one raw sample through validation, calibration, filtering,
        // orientation, and world-frame kinematics.
        // Returns true when a valid KinematicSample is produced in `output`.
        bool process(const RawImuSample& sample, KinematicSample& output);

        void reset();

        ImuCalibrator::State calibrationState() const { return calibrator_.state(); }

        const PipelineCounters& counters() const { return counters_; }

    private:
        bool validateAndTrackSequence(const RawImuSample& sample);

        uint32_t expectedSequence_{0};
        bool havePreviousSequence_{false};

        ImuCalibrator calibrator_{};
        ImuFilter filter_{};
        OrientationEstimator orientation_{};
        KinematicsEstimator kinematics_{};

        PipelineCounters counters_{};
    };
}
