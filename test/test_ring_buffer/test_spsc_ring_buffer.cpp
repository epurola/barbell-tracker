
#include <unity.h>
#include <Arduino.h>

#include "contracts/ImuTypes.h"
#include "runtime/SpscRingBuffer.h"

using vbt::RawImuSample;
using vbt::runtime::SpscRingBuffer;

static SpscRingBuffer<RawImuSample, 8> queue;

void setUp()
{
    RawImuSample discarded{};
    while (queue.tryPop(discarded)) {}
}

void tearDown() {}

void test_empty_queue_returns_false()
{
    RawImuSample sample{};

    TEST_ASSERT_FALSE(queue.tryPop(sample));
    TEST_ASSERT_EQUAL_UINT32(0, queue.sizeApprox());
}

void test_fifo_order_is_preserved()
{
    RawImuSample first{};
    first.timestamp_us = 1000;
    first.sequence = 1;

    RawImuSample second{};
    second.timestamp_us = 2000;
    second.sequence = 2;

    TEST_ASSERT_TRUE(queue.tryPush(first));
    TEST_ASSERT_TRUE(queue.tryPush(second));

    RawImuSample output{};

    TEST_ASSERT_TRUE(queue.tryPop(output));
    TEST_ASSERT_EQUAL_UINT32(1, output.sequence);
    TEST_ASSERT_EQUAL_UINT64(1000, output.timestamp_us);

    TEST_ASSERT_TRUE(queue.tryPop(output));
    TEST_ASSERT_EQUAL_UINT32(2, output.sequence);
    TEST_ASSERT_EQUAL_UINT64(2000, output.timestamp_us);

    TEST_ASSERT_FALSE(queue.tryPop(output));
}

void test_full_queue_rejects_new_sample()
{
    RawImuSample input{};

    for (uint32_t i = 0; i < 8; ++i)
    {
        input.sequence = i;
        TEST_ASSERT_TRUE(queue.tryPush(input));
    }

    input.sequence = 99;

    TEST_ASSERT_FALSE(queue.tryPush(input));
    TEST_ASSERT_EQUAL_UINT32(1, queue.droppedCount());
    TEST_ASSERT_EQUAL_UINT32(8, queue.sizeApprox());

    RawImuSample output{};
    for (uint32_t i = 0; i < 8; ++i)
    {
        TEST_ASSERT_TRUE(queue.tryPop(output));
        TEST_ASSERT_EQUAL_UINT32(i, output.sequence);
    }

    TEST_ASSERT_FALSE(queue.tryPop(output));
}

void test_queue_can_be_reused_after_pop()
{
    RawImuSample input{};
    RawImuSample output{};

    for (uint32_t i = 0; i < 8; ++i)
    {
        input.sequence = i;
        TEST_ASSERT_TRUE(queue.tryPush(input));
    }

    TEST_ASSERT_TRUE(queue.tryPop(output));

    input.sequence = 42;
    TEST_ASSERT_TRUE(queue.tryPush(input));
}

void setup()
{
    delay(3000);
    UNITY_BEGIN();

    RUN_TEST(test_empty_queue_returns_false);
    RUN_TEST(test_fifo_order_is_preserved);
    RUN_TEST(test_full_queue_rejects_new_sample);
    RUN_TEST(test_queue_can_be_reused_after_pop);

    UNITY_END();
}

void loop() {}

