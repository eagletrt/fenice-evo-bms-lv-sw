#include "unity.h"

#include "bms_lv_config.h"

#include <stdio.h>
#include <time.h>

float lv_voltages[CELL_COUNT];
float lv_temperatures[TEMP_SENSOR_COUNT];

void set_relay(uint8_t status) {
    status ? printf("Relay opened\n") : printf("Relay closed\n");
}

uint32_t get_current_time_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

void monitor_get_voltages(float *voltages) {
    voltages = (float *)lv_voltages;
}

void monitor_get_temperatures(float *temperatures) {
    temperatures = (float *)lv_temperatures;
}

void buzzer_beep_async(uint32_t buzzer_duration, buzzer_mode_t sound_mode) {
    printf("Buzzer beep async for %u\n", buzzer_duration);
}

uint8_t get_health_status(float i_bat, float i_chg, float bat_out, float relay_out, float lvms_out);
int check_status(uint8_t current_status);

float mux_fb_mV[mux_fb_n_values];
float mux_sensors_mA[mux_sensors_n_values];
float dc_fb_mV[directly_connected_fbs_n_values];
uint8_t mcp23017_feedbacks_state[8];

void setUp(void) {
}

void tearDown(void) {
}

void test_status_update() {
    float i_bat     = 60.0f;
    float i_chg     = 5000.0f;
    float bat_out   = 2000.0f;
    float relay_out = 3000.0f;
    float lvms_out  = 4000.0f;

    uint8_t current_status = get_health_status(i_bat, i_chg, bat_out, relay_out, lvms_out);
    TEST_ASSERT_EQUAL_UINT8(0b111011, current_status);
}

void health_status_case_0(void) {
    uint8_t current_status = 0b000000;
    TEST_ASSERT_EQUAL_UINT8(HEALTH_ERROR_STATUS, check_status(current_status));
}

void health_status_case_1(void) {
    uint8_t current_status = 0b000000;
    current_status         = 0b001100;
    TEST_ASSERT_EQUAL_UINT8(HEALTH_ERROR_STATUS, check_status(current_status));
}

void health_status_case_2(void) {
    uint8_t current_status = 0b000000;
    current_status         = 0b000010;
    TEST_ASSERT_EQUAL_UINT8(HEALTH_SAFE_STATUS, check_status(current_status));
}

void health_status_case_3(void) {
    uint8_t current_status = 0b000000;
    current_status         = 0b000100;
    TEST_ASSERT_EQUAL_UINT8(HEALTH_SAFE_STATUS, check_status(current_status));
}

void health_status_case_4(void) {
    uint8_t current_status = 0b000000;
    current_status         = 0b001011;
    TEST_ASSERT_EQUAL_UINT8(HEALTH_SAFE_STATUS, check_status(current_status));
}

void health_status_case_5(void) {
    uint8_t current_status = 0b000000;
    current_status         = 0b001110;
    TEST_ASSERT_EQUAL_UINT8(HEALTH_SAFE_STATUS, check_status(current_status));
}

void health_status_case_6(void) {
    uint8_t current_status = 0b000000;
    current_status         = 0b001111;
    TEST_ASSERT_EQUAL_UINT8(HEALTH_SAFE_STATUS, check_status(current_status));
}

void health_status_case_7(void) {
    uint8_t current_status = 0b000000;
    current_status         = 0b011110;
    TEST_ASSERT_EQUAL_UINT8(HEALTH_SAFE_STATUS, check_status(current_status));
}

void health_status_case_8(void) {
    uint8_t current_status = 0b000000;
    current_status         = 0b011111;
    TEST_ASSERT_EQUAL_UINT8(HEALTH_SAFE_STATUS, check_status(current_status));
}

void health_status_case_9(void) {
    uint8_t current_status = 0b000000;
    current_status         = 0b110111;
    TEST_ASSERT_EQUAL_UINT8(HEALTH_SAFE_STATUS, check_status(current_status));
}

void health_status_case_10(void) {
    uint8_t current_status = 0b000000;
    current_status         = 0b111111;
    TEST_ASSERT_EQUAL_UINT8(HEALTH_SAFE_STATUS, check_status(current_status));
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_status_update);
    RUN_TEST(health_status_case_0);
    RUN_TEST(health_status_case_1);
    RUN_TEST(health_status_case_2);
    RUN_TEST(health_status_case_3);
    RUN_TEST(health_status_case_4);
    RUN_TEST(health_status_case_5);
    RUN_TEST(health_status_case_6);
    RUN_TEST(health_status_case_7);
    RUN_TEST(health_status_case_8);
    RUN_TEST(health_status_case_9);
    RUN_TEST(health_status_case_10);

    UNITY_END();
}