#pragma once

#include <cstdint>

#include "contracts/FirmwareContracts.h"
#include "processing/ActivityAnalyzer.h"
#include "processing/BarbellRepAnalyzer.h"
#include "processing/BlockStartAnalyzer.h"
#include "processing/HurdleContactAnalyzer.h"
#include "processing/ImuCalibration.h"
#include "processing/ImuFilter.h"
#include "processing/KinematicsEstimator.h"
#include "processing/OrientationEstimator.h"
#include "processing/SprintContactAnalyzer.h"
#include "processing/StandingJumpanalyzer.h"

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
        uint32_t completedEfforts{0};
    };

    class ImuPipeline
    {
    public:
        ImuPipeline();

        bool process(const RawImuSample& sample, KinematicSample& kinematicsOut, ActivityOutput& activityOut);

        void reset();

        // Starts a manual recording session in `mode`.
        // If `recalibrate` is true (or calibration has not completed yet), runs a 1-second
        // stationary calibration first (`SessionState::Calibrating` -> `SessionState::Recording`).
        // If `recalibrate` is false and already calibrated (e.g. tapping `[ NEXT SET ]`),
        // resets the analyzer for the next set and starts `SessionState::Recording` immediately.
        void startSession(ActivityMode mode, bool recalibrate = true);

        // Starts the next set in the current mode without recalibrating gyro bias.
        void startNextSet();

        // Ends the current recording window, flushes any in-progress effort, and enters `ResultsReady`.
        void stopSession(ActivityOutput& finalOutput);

        // Returns to the swipeable mode selection carousel (`IdleModeSelect`).
        void returnToModeSelect();

        void setMode(ActivityMode mode);
        ActivityMode mode() const { return activeAnalyzer_->mode(); }
        SessionState sessionState() const { return sessionState_; }
        uint32_t setNumber() const { return setNumber_; }
        bool isEffortInProgress() const { return activeAnalyzer_->isEffortInProgress(); }
        uint8_t activePhaseCode() const { return activeAnalyzer_->phaseCode(); }

        ImuCalibrator::State calibrationState() const { return calibrator_.state(); }
        const PipelineCounters& counters() const { return counters_; }
        const ActivityOutput& lastCompletedActivity() const { return lastCompletedActivity_; }

    private:
        bool validateAndTrackSequence(const RawImuSample& sample);
        ActivityAnalyzer* analyzerForMode(ActivityMode mode);

        uint32_t expectedSequence_{0};
        bool havePreviousSequence_{false};

        SessionState sessionState_{SessionState::Recording};
        uint32_t setNumber_{1};

        ImuCalibrator calibrator_{};
        ImuFilter filter_{};
        OrientationEstimator orientation_{};
        KinematicsEstimator kinematics_{};

        BarbellRepAnalyzer barbellAnalyzer_{};
        StandingJumpAnalyzer standingJumpAnalyzer_{};
        HurdleContactAnalyzer hurdleAnalyzer_{};
        SprintContactAnalyzer sprintAnalyzer_{};
        BlockStartAnalyzer blockStartAnalyzer_{};
        ActivityAnalyzer* activeAnalyzer_{nullptr};

        ActivityOutput lastCompletedActivity_{};
        PipelineCounters counters_{};
    };
}
