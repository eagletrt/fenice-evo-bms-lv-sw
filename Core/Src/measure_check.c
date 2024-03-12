#include "bms_lv_config.h"
#include "math.h"
#include <stdint.h>

void set_relay(uint8_t status);

extern float mux_fb_mV[mux_fb_n_values];
extern float mux_sensors_mA[mux_sensors_n_values];
extern float dc_fb_mV[directly_connected_fbs_n_values];
extern uint8_t mcp23017_feedbacks_state[8];

// Index is from right to left
void set_bit(uint8_t *value, uint8_t index, uint8_t bit_value) {
  if (bit_value == 0) {
    *value = *value & ~(1 << index);
  } else if (bit_value == 1) {
    *value = *value | (1 << index);
  }
}

uint8_t convert_relay_out_ang_to_state(float relay_out, float bat_out) {
  uint8_t state = 0;
  float diff_voltage = relay_out - bat_out;

  if (diff_voltage < 0) {
    diff_voltage = -diff_voltage;
  }

  if (diff_voltage < MIN_RELAY_VOLTAGE_DIFF_THRESHOLD_mV &&
      relay_out > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV &&
      bat_out > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV) {
    state = 1;
  } else {
    state = 0;
  }

  return state;
}

uint8_t convert_lvms_out_ang_to_state(float relay_out, float lvms_out) {
  uint8_t state = 0;
  float diff_voltage = relay_out - lvms_out;

  if (diff_voltage < 0) {
    diff_voltage = -diff_voltage;
  }

  if (diff_voltage < MIN_LVMS_VOLTAGE_DIFF_THRESHOLD_mV &&
      relay_out > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV &&
      lvms_out > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV) {
    state = 1;
  } else {
    state = 0;
  }

  return state;
}

void update_status(uint8_t *current_status, float i_bat, float i_chg,
                   float bat_out, float relay_out, float lvms_out) {
  set_bit(current_status, 5, i_bat < 0 ? 0 : 1);
  set_bit(current_status, 4, i_bat < MIN_BATTERY_CURRENT_THRESHOLD_mA ? 0 : 1);
  set_bit(current_status, 3, i_chg < MIN_CHARGER_CURRENT_THRESHOLD_mA ? 0 : 1);
  set_bit(current_status, 2, bat_out < MIN_BATTERY_VOLTAGE_mV ? 0 : 1);
  set_bit(current_status, 1,
          convert_relay_out_ang_to_state(relay_out, bat_out));
  set_bit(current_status, 0,
          convert_lvms_out_ang_to_state(relay_out, lvms_out));
}

int check_status(uint8_t current_status) {
  switch (current_status) {
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