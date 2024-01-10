#ifndef BMS_LV_CONFIG_H
#define BMS_LV_CONFIG_H

#include <stdbool.h>

#define ADC_HALL_AND_FB               &hadc2
#define ADC_VREF_CALIBRATION          &hadc1
#define TIMER_ADC_CALIBRATION         &htim1
#define TIMER_ADC_ROUTINE             &htim10
#define TIMER_ADC_CALIBRATION_CHANNEL TIM_CHANNEL_1

#define HISTORY_L                  25
#define ADC2_CHANNELS_N            6
#define MUX_CHANNELS_N             16
#define ADC_CALIBRATION_CHANNELS_N 2
#define ADC_CALIBRATION_SAMPLES_N  500

// Directly connected LV Feedbacks (no mux)
#define FB_COMPUTER_FB_IDX       0
#define FB_RELAY_OUT_IDX         1
#define FB_LVMS_OUT_IDX          2
#define FB_BATT_OUT_IDX          3
#define N_DIRECTLY_CONNECTED_FBS 4

#define MEAN_VALUES_ARRAY_LEN           ((MUX_CHANNELS_N * 2) + 4)  // (multiplexer channels * 2) + 4 adc2_channels
#define S_HALL0                         1
#define S_HALL1                         3
#define S_HALL2                         5
#define S_HALL1_OFFSET_mA               300.0f
#define S_HALL2_OFFSET_mA               1500.0f
#define ADC2_VOLTAGE_DIVIDER_MULTIPLIER 9.0f

enum multiplexer_channel_addresses {
    multiplexer_channel_0,
    multiplexer_channel_8,
    multiplexer_channel_4,
    multiplexer_channel_12,
    multiplexer_channel_2,
    multiplexer_channel_10,
    multiplexer_channel_6,
    multiplexer_channel_14,
    multiplexer_channel_1,
    multiplexer_channel_9,
    multiplexer_channel_5,
    multiplexer_channel_13,
    multiplexer_channel_3,
    multiplexer_channel_11,
    multiplexer_channel_7,
    multiplexer_channel_15
};

enum ADC2_channels {
    adc2_channel_mux_hall,
    adc2_channel_mux_fb,
    adc2_channel_adcs_as_computer_fb,
    adc2_channel_adcs_relay_out,
    adc2_channel_adcs_lvms_out,
    adc2_channel_adcs_batt_out,
};

enum feedbacks_values {
    feedbacks_value_sd_end,
    feedbacks_value_bspd_fb,
    feedbacks_value_imd_fb,
    feedbacks_value_lvms_fb,
    feedbacks_value_res_fb,
    feedbacks_value_pin5_disconnected,
    feedbacks_value_lv_encl,
    feedbacks_value_pin7_disconnected,
    feedbacks_value_hv_encl_1_fb,
    feedbacks_value_hv_encl_2_fb,
    feedbacks_value_back_plate_fb,
    feedbacks_value_hvd_fb,
    feedbacks_value_ams_fb,
    feedbacks_value_asms_fb,
    feedbacks_value_interlock_fb,
    feedbacks_value_sd_start
};

enum sensors_values {
    sensors_values_hall_0cd0,
    sensors_values_s_hall0,
    sensors_values_hall_0cd1,
    sensors_values_s_hall1,
    sensors_values_hall_0cd2,
    sensors_values_s_hall2,
    sensors_values_lvac_temp0,
    sensors_values_lvac_temp1,
    sensors_values_pin8_disconnected,
    sensors_values_pin9_disconnected,
    sensors_values_pin10_disconnected,
    sensors_values_pin11_disconnected,
    sensors_values_pin12_disconnected,
    sensors_values_pin13_disconnected,
    sensors_values_pin14_disconnected,
    sensors_values_pin15_disconnected
};

#endif  // BMS_LV_CONFIG_H
