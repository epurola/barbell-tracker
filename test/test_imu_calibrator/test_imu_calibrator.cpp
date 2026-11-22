
#include <unity.h>

#include "processing/ImuCalibration.h"

using namespace vbt;
using namespace vbt::processing;

namespace
{
    RawImuSample makeStationarySample(uint32_t sequence)
    {
        RawImuSample sample{};

        sample.timestamp_us =
            static_cast<uint64_t>(sequence) * 2000;

        sample.sequence = sequence;

        // Stationary sensor, with a small fixed gyro bias.
        sample.accelerometer = Vector3(0.0f, 0.0f, 9.80665f);
        sample.gyroscope = Vector3(0.01f, -0.02f, 0.03f);

        sample.status =
            SampleStatus::Valid |
            SampleStatus::AccelorometerOk |
            SampleStatus::GyroscopeOk |
            SampleStatus::TimestampValid |
            SampleStatus::SequenceValid;

        return sample;
    }
}

void test_stationary_calibration_completes()
{
    ImuCalibrator calibrator;

    for (uint32_t i = 0;
         i < ImuCalibrator::CALIBRATION_SAMPLE_COUNT;
         ++i)
    {
        calibrator.collect(makeStationarySample(i));
    }

    TEST_ASSERT_EQUAL(
        static_cast<int>(ImuCalibrator::State::Ready),
        static_cast<int>(calibrator.state()));

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.01f,
        calibrator.gyroscopeBiasRadps().x);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        -0.02f,
        calibrator.gyroscopeBiasRadps().y);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.03f,
        calibrator.gyroscopeBiasRadps().z);
}

void test_calibration_rejects_motion()
{
    ImuCalibrator calibrator;

    for (uint32_t i = 0;
         i < ImuCalibrator::CALIBRATION_SAMPLE_COUNT;
         ++i)
    {
        RawImuSample sample = makeStationarySample(i);

        // Excessive angular movement.
        sample.gyroscope.x = 0.5f;

        calibrator.collect(sample);
    }

    TEST_ASSERT_EQUAL(
        static_cast<int>(ImuCalibrator::State::Failed),
        static_cast<int>(calibrator.state()));
}

void test_apply_removes_gyro_bias()
{
    ImuCalibrator calibrator;

    for (uint32_t i = 0;
         i < ImuCalibrator::CALIBRATION_SAMPLE_COUNT;
         ++i)
    {
        calibrator.collect(makeStationarySample(i));
    }

    RawImuSample raw = makeStationarySample(501);

    raw.gyroscope = Vector3(0.11f, 0.08f, 0.13f);

    CalibratedImuSample calibrated{};

    TEST_ASSERT_TRUE(calibrator.apply(raw, calibrated));

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.10f,
        calibrated.gyroscope_radps.x);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.10f,
        calibrated.gyroscope_radps.y);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.10f,
        calibrated.gyroscope_radps.z);

    TEST_ASSERT_TRUE(
        hasStatus(calibrated.status, SampleStatus::Calibrated));
}

void setup()
{
    UNITY_BEGIN();

    RUN_TEST(test_stationary_calibration_completes);
    RUN_TEST(test_calibration_rejects_motion);
    RUN_TEST(test_apply_removes_gyro_bias);

    UNITY_END();
}

void loop()
{
}
