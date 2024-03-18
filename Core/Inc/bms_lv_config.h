#ifndef BMS_LV_CONFIG_H
#define BMS_LV_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define clamp(num, max_n, min_n)                                               \
  ((num > max_n) ? max_n : ((num < min_n) ? min_n : num))

#define MIN_CHARGER_CURRENT_THRESHOLD_mA 4000.0f
#define MIN_BATTERY_CURRENT_THRESHOLD_mA 50.0f
#define MIN_BATTERY_VOLTAGE_mV 3300.0 * 6.0f
// Min difference threshold between V Relay and V Battery
#define MIN_RELAY_VOLTAGE_DIFF_THRESHOLD_mV                                    \
  2000.0f // diff v relay (that could be the charger one and bat out)
// Min difference threshold between LVMS out V ans V Relay
#define MIN_LVMS_VOLTAGE_DIFF_THRESHOLD_mV                                     \
  2000.0f // diff lvms out and relay out 5%
#define MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV 500.0f // 500 mV

#define FLOAT_UNINITIALIZED_VALUE -1.0f
#define MCP23017_INTERRUPTS_ENABLED 1

#define SAFE_STATUS 0
#define ERR_STATUS 1

// Directly connected LV Feedbacks (no mux)
enum directly_connected_feedbacks_indexes {
  fb_computer_fb_idx = 0,
  fb_relay_out_idx,
  fb_lvms_out_idx,
  fb_batt_out_idx,
  directly_connected_fbs_n_values,
};

enum adc2_channel_indexes {
  adc2_ch_mux_hall_idx = 0,
  adc2_ch_mux_fb_idx,
  adc2_ch_not_used_idx,
  adc2_ch_relay_out_idx,
  adc2_ch_lvms_out_idx,
  adc2_ch_batt_out_idx,
  adc2_ch_n_values
};

enum mux_fb_indexes {
  mux_fb_sd_end_idx = 0,
  mux_fb_bspd_fb_idx,
  mux_fb_imd_fb_idx,
  mux_fb_lvms_fb_idx,
  mux_fb_res_fb_idx,
  mux_fb_pin5_disconnected_idx,
  mux_fb_lv_encl_idx,
  mux_fb_pin7_disconnected_idx,
  mux_fb_hv_encl_1_fb_idx,
  mux_fb_hv_encl_2_fb_idx,
  mux_fb_back_plate_fb_idx,
  mux_fb_hvd_fb_idx,
  mux_fb_ams_fb_idx,
  mux_fb_asms_fb_idx,
  mux_fb_interlock_fb_idx,
  mux_fb_sd_start_idx,
  mux_fb_n_values
};

enum mux_sensors_indexes {
  mux_sensors_hall_0cd0_idx = 0,
  mux_sensors_s_hall0_idx,
  mux_sensors_hall_0cd1_idx,
  mux_sensors_s_hall1_idx,
  mux_sensors_hall_0cd2_idx,
  mux_sensors_s_hall2_idx,
  mux_sensors_lvac_temp0_idx,
  mux_sensors_lvac_temp1_idx,
  mux_sensors_pin8_nc_idx,
  mux_sensors_pin9_nc_idx,
  mux_sensors_pin10_nc_idx,
  mux_sensors_pin11_nc_idx,
  mux_sensors_pin12_nc_idx,
  mux_sensors_pin13_nc_idx,
  mux_sensors_pin14_nc_idx,
  mux_sensors_pin15_nc_idx,
  mux_sensors_n_values
};

enum mcp_feedbacks_bank_a {
  mcp_feedbacks_bank_a_inverters_fb = 0,
  mcp_feedbacks_bank_a_pcbs_fb,
  mcp_feedbacks_bank_a_pumps_fb,
  mcp_feedbacks_bank_a_shutdown_fb,
  mcp_feedbacks_bank_a_radiators_fb,
  mcp_feedbacks_bank_a_fan_fb,
  mcp_feedbacks_bank_a__unused1,
  mcp_feedbacks_bank_a__unused2,
  mcp_feedbacks_bank_a_n_pins
};

enum mcp_controls_bank_b {
  mcp_controls_bank_b_led_0 = 0,
  mcp_controls_bank_b_led_1,
  mcp_controls_bank_b_led_2,
  mcp_controls_bank_b_frg,
  mcp_controls_bank_b_rfe,
  mcp_controls_bank_b__unused1,
  mcp_controls_bank_b_discharge,
  mcp_controls_bank_b__unused2,
  mcp_controls_bank_b_n_pins
};

enum safe_statuses {
  safe_statuses_chg_connect_only_mcu_pw = 0b000010,
  safe_statuses_mcu_on_bat_only = 0b000100,
  safe_statuses_car_on_chg_only = 0b001011,
  safe_statuses_chg_batt = 0b001110,
  safe_statuses_car_on_bat_only = 0b001111,
  safe_statuses_relay_closed_chg_car_keep_closed = 0b011110,
  safe_statuses_car_on_chg_and_bat = 0b011111,
  safe_statuses_car_running = 0b110111,
  safe_statuses_car_on_chg_and_bat_duplicated = 0b111111,
};

// coefficient for conversion formula from voltage input to temperature
#define TEMP_CONV_CONST_a 127.02004615145405
#define TEMP_CONV_CONST_b -0.06979687590434158
#define TEMP_CONV_CONST_c 2.1026948971763155e-05
#define TEMP_CONV_CONST_d -3.3042552498294913e-09
#define TEMP_CONV_CONST_e 1.3552262617901958e-13

#endif // BMS_LV_CONFIG_H