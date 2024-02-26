#include "unity.h"

//this should go in the dedicate config file
#define CHECK_OK 0
#define CHECK_ERR 1

void setUp(void) {

}

void tearDown(void) {

}

void check_all_measurements_check(void) {
    uint8_t status = 0b000000;

    TEST_ASSERT_EQUAL_UINT8(CHECK_ERR, check_status(status));
}

//clang test.c -I ../Lib/micro-libs/Unity/src ../Src/measure_check.c ../Inc/bms_lv_config.h 
int main() {
    UNITY_BEGIN();

    RUN_TEST(check_all_measurements_check);

    UNITY_END();
}