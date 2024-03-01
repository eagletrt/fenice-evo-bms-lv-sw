#include "bms_lv_config.h"
#include <stdint.h>
#include "math.h"

#define CONVERT_I_BAT_ANG_TO_DIG(value) (((value) < 0.0 || (value) > 0.0) ? 1 : 0)
#define CONVERT_ANG_TO_DIG(value) (((value) > 0.0) ? 1 : 0)

extern float mux_fb_mV[mux_fb_n_values];
extern float mux_sensors_mA[mux_sensors_n_values];
extern float dc_fb_mV[directly_connected_fbs_n_values];
extern uint8_t mcp23017_feedbacks_state[8];

// Index is from right to left
void set_bit(uint8_t* value, uint8_t index, uint8_t bit_value) {
    if (bit_value == 0) {
        *value = *value & ~(1 << index);
    } else if (bit_value == 1) {
        *value = *value | (1 << index);
    }
}

void update_status(uint8_t* current_status, float i_bat, float i_chg, float bat_out, float relay_out, float lvms_out) {  
    set_bit(current_status, 5, CONVERT_ANG_TO_DIG(i_bat));
    set_bit(current_status, 4, CONVERT_I_BAT_ANG_TO_DIG(i_bat));
    set_bit(current_status, 3, CONVERT_ANG_TO_DIG(i_chg));
    set_bit(current_status, 2, CONVERT_ANG_TO_DIG(bat_out));
    set_bit(current_status, 1, CONVERT_ANG_TO_DIG(relay_out));
    set_bit(current_status, 0, CONVERT_ANG_TO_DIG(lvms_out));
}

int check_status(uint8_t current_status) {
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
        return ERR_STATUS;
        break;
    }

    return SAFE_STATUS;
}

float i_bat, i_chg, bat_out, relay_out, lvms_out = 0;
uint8_t current_status = 0b000000;
uint8_t check_result = ERR_STATUS;

void all_measurements_check(void) {
    i_bat = mux_sensors_mA[mux_sensors_s_hall1_idx];
    i_chg = mux_sensors_mA[mux_sensors_s_hall2_idx];
    bat_out = dc_fb_mV[fb_batt_out_idx];
    relay_out = dc_fb_mV[fb_relay_out_idx];
    lvms_out = dc_fb_mV[fb_lvms_out_idx];

    update_status(&current_status, i_bat, i_chg, bat_out, relay_out, lvms_out);
    check_result = check_status(current_status);
    if (check_result != SAFE_STATUS) {
        // TO-DO: generate error
    }
}