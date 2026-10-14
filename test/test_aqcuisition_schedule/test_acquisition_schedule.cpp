#include <unity.h>
#include <Arduino.h>

#include "config/RuntimeConfig.h"
#include "runtime/AcquisitionSchedule.h"

using namespace vbt::runtime;

void test_period_is_2000_microseconds()
{
    TEST_ASSERT_EQUAL_UINT32(2000, ACQUISITION_PERIOD_US);
}

void test_first_deadline_is_one_period_after_epoch()
{
    TEST_ASSERT_EQUAL_UINT64(12000,acquisitionDeadlineUs(10000, 1, 2000));
}

void test_deadlines_advance_without_relative_delay_drift()
{
    TEST_ASSERT_EQUAL_UINT64(16000,acquisitionDeadlineUs(10000, 3, 2000));
}

void test_single_notification_has_no_missed_deadline()
{
    TEST_ASSERT_EQUAL_UINT32(0, missedNotifications(1));
}

void test_accumulated_notifications_count_missed_opportunities()
{
    TEST_ASSERT_EQUAL_UINT32(0, missedNotifications(0));
    TEST_ASSERT_EQUAL_UINT32(1, missedNotifications(2));
    TEST_ASSERT_EQUAL_UINT32(4, missedNotifications(5));
}

void setup()
{
    delay(3000);
    UNITY_BEGIN();

    RUN_TEST(test_period_is_2000_microseconds);
    RUN_TEST(test_first_deadline_is_one_period_after_epoch);
    RUN_TEST(test_deadlines_advance_without_relative_delay_drift);
    RUN_TEST(test_single_notification_has_no_missed_deadline);
    RUN_TEST(test_accumulated_notifications_count_missed_opportunities);

    UNITY_END();
}

void loop() {}
