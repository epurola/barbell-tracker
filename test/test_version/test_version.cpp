#include <unity.h>

#include "firmware/Version.h"

void test_firmware_version(void){
    TEST_ASSERT_EQUAL_INT(0, vbt::FIRMWARE_MAJOR);
    TEST_ASSERT_EQUAL_INT(1, vbt::FIRMWARE_MINOR);
    TEST_ASSERT_EQUAL_INT(0, vbt::FIRMWARE_PATCH);
}

void setup(){
    //delay(2000);

    UNITY_BEGIN();

    RUN_TEST(test_firmware_version);

    UNITY_END();

}

void loop(){

}