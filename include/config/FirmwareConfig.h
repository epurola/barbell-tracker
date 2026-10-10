#pragma once

namespace vbt
{

/**
 * Sensor-to-body mounting transform.
 *
 * This is deliberately represented as a 3x3 rotation matrix.
 * It allows the physical IMU mounting orientation to be changed
 * without modifying DSP/AHRS/kinematic algorithms.
 */
struct SensorToBodyTransform
{
    float m[3][3];
};

constexpr SensorToBodyTransform SENSOR_TO_BODY_IDENTITY =
{
    {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    }
};

}