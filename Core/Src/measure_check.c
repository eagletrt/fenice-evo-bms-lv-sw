#include "bms_lv_config.h"
#include <stdint.h>
#include "math.h"

void set_relay(uint8_t status);
#define CONVERT_ANG_TO_DIG(value) value < 0.0 || value > 0.0 ? 1 : 0
#define CHECK_OK 0
#define CHECK_ERR 1

extern float mux_fb_mV[mux_fb_n_values];
extern float mux_sensors_mA[mux_sensors_n_values];
extern float dc_fb_mV[directly_connected_fbs_n_values];
extern uint8_t mcp23017_feedbacks_state[8];

uint8_t current_status = 0;

// Index is from right to left
void set_bit(uint8_t* value, uint8_t index, uint8_t bit_value) {
    if (bit_value == 0) {
        *value = *value & ~(1 << index);
    } else if (bit_value == 1) {
        *value = *value | (1 << index);
    }
}

void update_status() {
    set_bit(&current_status, 5, signbit(mux_sensors_mA[mux_sensors_s_hall1_idx]) ? 0 : 1); // sign of I_BAT
    set_bit(&current_status, 4, CONVERT_ANG_TO_DIG(mux_sensors_mA[mux_sensors_s_hall1_idx])); // I_BAT
    set_bit(&current_status, 3, CONVERT_ANG_TO_DIG(mux_sensors_mA[mux_sensors_s_hall2_idx])); // I_CHG
    set_bit(&current_status, 2, CONVERT_ANG_TO_DIG(dc_fb_mV[fb_batt_out_idx])); // BAT_OUT
    set_bit(&current_status, 1, CONVERT_ANG_TO_DIG(dc_fb_mV[fb_relay_out_idx])); // RELAY_OUT
    set_bit(&current_status, 0, CONVERT_ANG_TO_DIG(dc_fb_mV[fb_lvms_out_idx])); // LVMS_OUT
}

int check_status() {
    switch (current_status)
    {
    case safe_statuses_chg_connect_only_mcu_pw:
        set_relay(0);
        break;
    case safe_statuses_mcu_on_bat_only:
        set_relay(1);
        break;
    case safe_statuses_car_on_chg_only:
        set_relay(0);
        break;
    case safe_statuses_chg_batt:
        set_relay(1);
        break;
    case safe_statuses_car_on_bat_only:
        set_relay(1);
        break;
    case safe_statuses_relay_closed_chg_car_keep_closed:
        set_relay(1);
        break;
    case safe_statuses_car_on_chg_and_bat:
        set_relay(1);
        break;
    case safe_statuses_car_running:
        set_relay(1);
        break;
    case safe_statuses_car_on_chg_and_bat_duplicated:
        set_relay(1);
        break;

    default:
        //TODO: generate error
        return CHECK_ERR;
        break;
    }

    return CHECK_OK;
}

void all_measurements_check(void) {
    update_status();
    uint8_t check_result = check_status();
    if (check_result != CHECK_OK) {
        // TO-DO: generate error
    }
}
