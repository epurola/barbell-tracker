#pragma once

#include <cstdint>
#include "contracts/RepTypes.h"

namespace vbt
{
    enum class ActivityMode : uint8_t
    {
        BarbellVbt = 0,
        StandingJump,
        HurdleContact,
        SprintContact,
        BlockStart
    };

    enum class SessionState : uint8_t
    {
        IdleModeSelect = 0, // Browsing swipeable mode cards; analyzers paused
        Calibrating,        // 1-second stationary calibration after pressing START
        Recording,          // Screen frozen; 500 Hz pipeline recording the set/run
        ResultsReady        // Set stopped; screen wakes up and shows full set results
    };

    struct JumpMetrics
    {
        static constexpr uint32_t MAX_SET_JUMPS = 6;

        uint32_t jump_number{0};
        uint64_t takeoff_timestamp_us{0};
        uint64_t landing_timestamp_us{0};

        float flight_time_ms{0.0f};
        float takeoff_velocity_mps{0.0f};
        float peak_velocity_mps{0.0f};
        float jump_height_cm{0.0f};

        // Set summary across multiple jumps
        float jump_heights_cm[MAX_SET_JUMPS]{};
        float best_jump_height_cm{0.0f};
        float mean_jump_height_cm{0.0f};

        bool valid{false};
    };

    struct HurdleContactMetrics
    {
        static constexpr uint32_t MAX_SET_HURDLES = 10;

        uint32_t contact_number{0};
        uint64_t touchdown_timestamp_us{0};
        uint64_t toeoff_timestamp_us{0};
        uint64_t next_landing_timestamp_us{0};

        float contact_time_ms{0.0f};
        float min_contact_time_ms{0.0f};
        float mean_contact_time_ms{0.0f};
        float flight_time_ms{0.0f};
        float jump_height_cm{0.0f};
        float reactive_strength_index{0.0f};
        float best_rsi{0.0f};
        float mean_rsi{0.0f};

        // Individual hurdle hop breakdown across the hurdle line
        float hop_contact_time_ms[MAX_SET_HURDLES]{};
        float hop_rsi[MAX_SET_HURDLES]{};

        bool valid{false};
    };

    struct SprintContactMetrics
    {
        uint32_t step_count{0};
        uint64_t sprint_start_timestamp_us{0};
        uint64_t sprint_end_timestamp_us{0};
        float last_contact_time_ms{0.0f};
        float mean_contact_time_ms{0.0f};
        float min_contact_time_ms{0.0f};
        float mean_flight_time_ms{0.0f};
        float step_frequency_hz{0.0f};
        float odd_even_asymmetry_percent{0.0f};
        float contact_drift_ms{0.0f};
        bool valid{false};
    };

    struct BlockStartMetrics
    {
        uint32_t step_count{0};
        float step_contact_time_ms[4]{0.0f, 0.0f, 0.0f, 0.0f};
        float step_pitch_deg[4]{0.0f, 0.0f, 0.0f, 0.0f};
        float set_pitch_deg{0.0f};
        float peak_drive_accel_mps2{0.0f};
        bool valid{false};
    };

    struct ActivityOutput
    {
        ActivityMode mode{ActivityMode::BarbellVbt};
        SessionState session_state{SessionState::IdleModeSelect};
        uint32_t set_number{0};

        // True while a lift/jump/hurdle/sprint is actively happening (freeze display updates).
        bool effort_in_progress{false};

        // True when a rep/jump/contact or manual set finishes.
        bool completed{false};

        RepMetrics rep{};
        JumpMetrics jump{};
        HurdleContactMetrics hurdle{};
        SprintContactMetrics sprint{};
        BlockStartMetrics block_start{};
    };
}
