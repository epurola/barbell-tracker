#include <unity.h>

#include "processing/ImuFilter.h"

using namespace vbt;
using namespace vbt::processing;

namespace
{
    CalibratedImuSample makeSample(
        uint32_t sequence,
        uint64_t timestampUs,
        float acceleration,
        float gyro)
    {
        CalibratedImuSample sample{};

        sample.timestamp_us = timestampUs;
        sample.sequence = sequence;

        sample.accelerometer_mps2 =
            Vector3(acceleration, acceleration, acceleration);

        sample.gyroscope_radps =
            Vector3(gyro, gyro, gyro);

        sample.status =
            SampleStatus::Valid |
            SampleStatus::AccelorometerOk |
            SampleStatus::GyroscopeOk |
            SampleStatus::TimestampValid |
            SampleStatus::SequenceValid |
            SampleStatus::Calibrated;

        return sample;
    }
}

void test_first_sample_initializes_filter()
{
    ImuFilter filter;

    const auto input = makeSample(1, 2000, 9.8f, 0.1f);

    FilteredImuSample output{};

    TEST_ASSERT_TRUE(filter.process(input, output));

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 9.8f, output.accelerometer_mps2.x);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.1f, output.gyroscope_radps.x);

    TEST_ASSERT_TRUE(
        hasStatus(output.status, SampleStatus::Filtered));
}

void test_filter_smooths_input_changes()
{
    ImuFilter filter;

    FilteredImuSample output{};

    auto first = makeSample(1, 2000, 0.0f, 0.0f);
    auto second = makeSample(2, 4000, 10.0f, 10.0f);

    TEST_ASSERT_TRUE(filter.process(first, output));
    TEST_ASSERT_TRUE(filter.process(second, output));

    TEST_ASSERT_TRUE(output.accelerometer_mps2.x > 0.0f);
    TEST_ASSERT_TRUE(output.accelerometer_mps2.x < 10.0f);

    TEST_ASSERT_TRUE(output.gyroscope_radps.x > 0.0f);
    TEST_ASSERT_TRUE(output.gyroscope_radps.x < 10.0f);
}

void test_filter_rejects_non_increasing_timestamp()
{
    ImuFilter filter;

    FilteredImuSample output{};

    auto first = makeSample(1, 2000, 1.0f, 1.0f);
    auto second = makeSample(2, 2000, 2.0f, 2.0f);

    TEST_ASSERT_TRUE(filter.process(first, output));
    TEST_ASSERT_FALSE(filter.process(second, output));
}

void test_filter_rejects_uncalibrated_input()
{
    ImuFilter filter;

    auto input = makeSample(1, 2000, 1.0f, 1.0f);

    input.status = SampleStatus::Valid;

    FilteredImuSample output{};

    TEST_ASSERT_FALSE(filter.process(input, output));
}

void setup()
{
    UNITY_BEGIN();

    RUN_TEST(test_first_sample_initializes_filter);
    RUN_TEST(test_filter_smooths_input_changes);
    RUN_TEST(test_filter_rejects_non_increasing_timestamp);
    RUN_TEST(test_filter_rejects_uncalibrated_input);

    UNITY_END();
}

void loop()
{
}