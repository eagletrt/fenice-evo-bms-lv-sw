#include "unity.h"
#include "bms_lv_config.h"

/**
 * @brief Mocking
 * 
*/
void set_relay(uint8_t status) {}

float mux_fb_mV[mux_fb_n_values];
float mux_sensors_mA[mux_sensors_n_values];
float dc_fb_mV[directly_connected_fbs_n_values];
uint8_t mcp23017_feedbacks_state[8];

void setUp(void) {

}

void tearDown(void) {

}

void test_i_bat_update() {
    uint8_t current_status = 0b000000;
    float i_bat = 0.0;
    float i_chg = 0.0;
    float bat_out = 0.0;
    float relay_out = 0.0;
    float lvms_out = 0.0;

    update_status(&current_status, i_bat, i_chg, bat_out, relay_out, lvms_out);
    TEST_ASSERT_EQUAL_UINT8(0b000000, current_status);

    i_bat = -1.5;
    update_status(&current_status, i_bat, i_chg, bat_out, relay_out, lvms_out);
    TEST_ASSERT_EQUAL_UINT8(0b010000, current_status);

    i_bat = +1.5;
    update_status(&current_status, i_bat, i_chg, bat_out, relay_out, lvms_out);
    TEST_ASSERT_EQUAL_UINT8(0b110000, current_status);

    i_bat = 0.0;
    update_status(&current_status, i_bat, i_chg, bat_out, relay_out, lvms_out);
    TEST_ASSERT_EQUAL_UINT8(0b000000, current_status);
}

void test_status_update() {
    uint8_t current_status = 0b000000;
    float i_bat = 0.0;
    float i_chg = 2.0;
    float bat_out = -5.0;
    float relay_out = 2.2;
    float lvms_out = 3.3;

    update_status(&current_status, i_bat, i_chg, bat_out, relay_out, lvms_out);
    TEST_ASSERT_EQUAL_UINT8(0b001011, current_status);
}

void test_check_status(void) {
    uint8_t current_status = 0b000000;
    TEST_ASSERT_EQUAL_UINT8(ERR_STATUS, check_status(current_status));

    current_status = 0b001100;
    TEST_ASSERT_EQUAL_UINT8(ERR_STATUS, check_status(current_status));

    current_status = 0b000010;
    TEST_ASSERT_EQUAL_UINT8(SAFE_STATUS, check_status(current_status));

    current_status = 0b000100;
    TEST_ASSERT_EQUAL_UINT8(SAFE_STATUS, check_status(current_status));
    
    current_status = 0b001011;
    TEST_ASSERT_EQUAL_UINT8(SAFE_STATUS, check_status(current_status));
    
    current_status = 0b001110;
    TEST_ASSERT_EQUAL_UINT8(SAFE_STATUS, check_status(current_status));
    
    current_status = 0b001111;
    TEST_ASSERT_EQUAL_UINT8(SAFE_STATUS, check_status(current_status));
    
    current_status = 0b011110;
    TEST_ASSERT_EQUAL_UINT8(SAFE_STATUS, check_status(current_status));
    
    current_status = 0b011111;
    TEST_ASSERT_EQUAL_UINT8(SAFE_STATUS, check_status(current_status));
    
    current_status = 0b110111;
    TEST_ASSERT_EQUAL_UINT8(SAFE_STATUS, check_status(current_status));
    
    current_status = 0b111111;
    TEST_ASSERT_EQUAL_UINT8(SAFE_STATUS, check_status(current_status));
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_i_bat_update);
    RUN_TEST(test_status_update);
    RUN_TEST(test_check_status);

    UNITY_END();
}