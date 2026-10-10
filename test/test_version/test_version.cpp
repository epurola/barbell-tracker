#include <unity.h>
#include <Arduino.h>
#include "firmware/Version.h"

void test_firmware_version(void) {
    TEST_ASSERT_EQUAL_INT(0, vbt::FIRMWARE_MAJOR);
    TEST_ASSERT_EQUAL_INT(1, vbt::FIRMWARE_MINOR);
    TEST_ASSERT_EQUAL_INT(0, vbt::FIRMWARE_PATCH);
}

void test_cpp_standard(void) {
    TEST_ASSERT_TRUE(__cplusplus >= 201703L);
}

void setup() {
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_firmware_version);
    RUN_TEST(test_cpp_standard);
    UNITY_END();
}

void loop() {}