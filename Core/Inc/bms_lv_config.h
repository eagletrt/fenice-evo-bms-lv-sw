#ifndef BMS_LV_CONFIG_H
#define BMS_LV_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define clamp(num, max_n, min_n)                                               \
  ((num > max_n) ? max_n : ((num < min_n) ? min_n : num))

#define UNINITIALIZED_VALUE -1.0f

// Directly connected LV Feedbacks (no mux)
enum directly_connected_feedbacks_indexes {
  fb_computer_fb_idx,
  fb_relay_out_idx,
  fb_lvms_out_idx,
  fb_batt_out_idx,
  directly_connected_fbs_n_values,
};

enum adc2_channel_indexes {
  adc2_ch_mux_hall_idx,
  adc2_ch_mux_fb_idx,
  adc2_ch_not_used_idx,
  adc2_ch_relay_out_idx,
  adc2_ch_lvms_out_idx,
  adc2_ch_batt_out_idx,
  adc2_ch_n_values
};

enum mux_fb_indexes {
  mux_fb_sd_end_idx,
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
  mux_sensors_hall_0cd0_idx,
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

#endif // BMS_LV_CONFIG_H
