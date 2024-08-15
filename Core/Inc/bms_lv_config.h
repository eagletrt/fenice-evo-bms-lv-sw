#ifndef BMS_LV_CONFIG_H
#define BMS_LV_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define clamp(num, max_n, min_n) ((num > max_n) ? max_n : ((num < min_n) ? min_n : num))

#define max(a, b) (a > b) ? a : b

#define CAN_MESSAGES_HANDLERS               \
    {primary_lv_set_radiator_speed_handler, \
     primary_lv_set_pumps_speed_handler,    \
     primary_hv_status_handler,             \
     inverters_inv_l_rcv_handler,           \
     inverters_inv_r_rcv_handler,           \
     primary_flash_request_handler,         \
     primary_flash_request_handler,         \
     primary_ecu_status_handler,            \
     primary_lv_cooling_aggressiveness_handler}

#define LV_VOLTAGE_CHECKS_DISABLED_TIMEOUT_ms (5000U)
#define MIN_CHARGER_CURRENT_THRESHOLD_mA      (2500.0f)
#define MIN_BATTERY_CURRENT_THRESHOLD_mA      (50.0f)
#define LVMS_THRESHOLD_mV                     (20000)
#define HEALTH_STATUS_CHECK_ENABLED           (0U)
#define WAIT_BEFORE_CHECKING_LVMS_ms          (2000U)

#define COOLING_TYPE_POLYNOMIAL (0u)
#define COOLING_TYPE_PID        (1u)

#define COOLING_TYPE COOLING_TYPE_POLYNOMIAL
// #define COOLING_TYPE COOLING_TYPE_PID

#if COOLING_TYPE == COOLING_TYPE_POLYNOMIAL
#define COOLING_AGGRESSIVENESS_LOW    (0u)
#define COOLING_AGGRESSIVENESS_NORMAL (1u)
#define COOLING_AGGRESSIVENESS_HIGH   (2u)
#define COOLING_AGGRESSIVENESS        COOLING_AGGRESSIVENESS_NORMAL
#endif

// Number of cells present in the bms lv, the cell configuration is 6s4p
#define CELL_COUNT (6)
// Number of ntc sensors present in the bms lv
#define TEMP_SENSOR_COUNT       (12)
#define MAX_CELL_VOLTAGE_V      (4.2f)
#define MIN_CELL_VOLTAGE_V      (3.2f)
#define MIN_BATTERY_VOLTAGE_mV  ((MIN_CELL_VOLTAGE_V * 1000.0f) * 6.0f)
#define MAX_CELL_TEMP           (60.0f)
#define MIN_CELL_TEMP           (8.0f)
#define WARNING_VOLTAGE_V       (3.3f)
#define BUZZER_WARNING_INTERVAL (60000U)

// Max current output allowed
#define MAX_CURRENT_mA (18000.0f)
// Min difference threshold between V Relay and V Battery
#define MIN_RELAY_VOLTAGE_DIFF_THRESHOLD_mV (2000.0f)  // diff v relay (that could be the charger one and bat out)
// Min difference threshold between LVMS out V ans V Relay
#define MIN_LVMS_VOLTAGE_DIFF_THRESHOLD_mV (2000.0f)  // diff lvms out and relay out 5%
#define MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV   (500.0f)   // 500 mV
// adc voltage divider value
#define ADC_VOLTAGE_DIVIDER_MULTIPLIER (9.0f)
// Threshold to check if an adc feedbacks is either logic high or logic low
#define HIGH_LEVEL_THRESHOLD (1000.f) * ADC_VOLTAGE_DIVIDER_MULTIPLIER

#define FLOAT_UNINITIALIZED_VALUE   (-1.0f)
#define MCP23017_INTERRUPTS_ENABLED (1u)

#define HEALTH_SAFE_STATUS  (0u)
#define HEALTH_ERROR_STATUS (1u)

#define WARNING_BUZZ_DURATION_ms (250u)

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
    mux_fb_hvd_fb_idx,
    mux_fb_lvms_fb_idx,
    mux_fb_res_fb_idx,
    mux_fb_pin5_disconnected_idx,
    mux_fb_lv_encl_fb_idx,
    mux_fb_pin7_disconnected_idx,
    mux_fb_invc_lid_fb_idx,
    mux_fb_hv_encl_2_fb_idx,
    mux_fb_backplate_fb_idx,
    mux_fb_invc_interlock_fb_idx,
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

#define HEALTH_IBAT_SIGN_INDEX (0U)
#define HEALTH_IBAT_INDEX      (1U)
#define HEALTH_ICHARGER_INDEX  (2U)
#define HEALTH_BATTOUT_INDEX   (3U)
#define HEALTH_RELAY_OUT_INDEX (4U)
#define HEALTH_LVMS_OUT_INDEX  (5U)
#define HEALTH_SIGNALS_N       (6U)

// #define HEALTH_SAFE_STATUS_CHG_CONNECT_ONLY_MCU_PW          (0b000010) // not safe (?)
#define HEALTH_SAFE_STATUS_MCU_ON_BAT_ONLY (0b000100)
// #define HEALTH_SAFE_STATUS_CAR_ON_CHG_ONLY                  (0b001011) // not safe (?)
#define HEALTH_SAFE_STATUS_CHG_BATT                         (0b001110)
#define HEALTH_SAFE_STATUS_CAR_ON_BAT_ONLY                  (0b001111)
#define HEALTH_SAFE_STATUS_RELAY_CLOSED_CHG_CAR_KEEP_CLOSED (0b011110)
#define HEALTH_SAFE_STATUS_CAR_ON_CHG_AND_BAT               (0b011111)
#define HEALTH_SAFE_STATUS_CAR_RUNNING                      (0b110111)
#define HEALTH_SAFE_STATUS_CAR_ON_CHG_AND_BAT_DUPLICATED    (0b111111)

enum {
    BMS_LV_PRIMARY_LV_SET_RADIATOR_SPEED = 0,
    BMS_LV_PRIMARY_LV_SET_PUMPS_SPEED,
    BMS_LV_PRIMARY_HV_STATUS,
    BMS_LV_INVERTERS_INV_L_RCV,
    BMS_LV_INVERTERS_INV_R_RCV,
    BMS_LV_PRIMARY_LV_CAN_FLASH_REQ_STEERING_WHEEL,
    BMS_LV_PRIMARY_LV_CAN_FLASH_REQ_TLM,
    BMS_LV_PRIMARY_ECU_STATUS,
    BMS_LV_PRIMARY_COOLING_AGGRESSIVENESS,
    N_MONITORED_MESSAGES
};

enum {
    LV_MSG_LV_STATUS_MSG_IDX = 0,
    LV_MSG_LV_INVERTER_CONNECTION_STATUS_MSG_IDX,
    LV_MSG_LV_ERRORS_MSG_IDX,
    LV_MSG_LV_CELLS_VOLTAGE_MSG_IDX,
    LV_MSG_LV_CELLS_TEMP_MSG_IDX,
    LV_MSG_LV_TOTAL_VOLTAGE_MSG_IDX,
    LV_MSG_LV_CURRENT_BATTERY_MSG_IDX,
    LV_MSG_LV_CURRENT_CHARGER_MSG_IDX,
    LV_MSG_LV_FEEDBACK_TS_VOLTAGE_MSG_IDX,
    LV_MSG_LV_FEEDBACK_SD_VOLTAGE_MSG_IDX,
    LV_MSG_LV_FEEDBACK_ENCLOSURE_VOLTAGE_MSG_IDX,
    LV_MSG_LV_FEEDBACK_GPIO_EXTENDER_MSG_IDX,
    LV_MSG_LV_FEEDBACK_MSG_IDX,
    LV_MSG_LV_PUMPS_SPEED_MSG_IDX,
    LV_MSG_LV_RADIATOR_SPEED_MSG_IDX,
    LV_MSG_LV_CHARGING_STATUS_MSG_IDX,
    LV_MSG_LV_CELLS_VOLTAGE_STATS_MSG_IDX,
    LV_MSG_LV_CELLS_TEMP_STATS_MSG_IDX,
    LV_MSG_LV_VERSION_MSG_IDX,
    LV_MSG_N_MSG_TO_SEND
};

typedef enum {
    BUZZER_MODE_NORMAL,
    BUZZER_MODE_WARNING,
} buzzer_mode_t;

typedef enum {
    BUZZER_SOUND,
    BUZZER_PAUSE,
} buzzer_sound_t;

#define WARNING_BUZZ_LENGTH (6U)

// coefficient for conversion formula from voltage input to temperature
#define TEMP_CONV_CONST_a (127.02004615145405)
#define TEMP_CONV_CONST_b (-0.06979687590434158)
#define TEMP_CONV_CONST_c (2.1026948971763155e-05)
#define TEMP_CONV_CONST_d (-3.3042552498294913e-09)
#define TEMP_CONV_CONST_e (1.3552262617901958e-13)

/* PUM RELATED STUFF */
#define PUMP_DAC    (hdac)
#define PUMP_L_CHNL (DAC_CHANNEL_1)
#define PUMP_R_CHNL (DAC_CHANNEL_2)

/* RAD_L AND RAD_R     -> TIM3 */
#define RAD_HTIM           (htim3)
#define RAD_L_PWM_TIM_CHNL (TIM_CHANNEL_1)
#define RAD_R_PWM_TIM_CHNL (TIM_CHANNEL_2)

/* BUZZER    -> TIM8 CH1 */
#define BZZR_HTIM         (htim8)
#define BZZR_TIMER        (htim10)
#define BZZR_PWM_TIM_CHNL (TIM_CHANNEL_1)

#define ERROR_TIMER htim7

uint32_t get_current_time_ms(void);
void blocking_delay_ms(uint32_t ms);
void monitor_get_voltages(float *);
void monitor_get_temperatures(float *);
int set_discharge(int state);
int set_rfe_frg(int state);
void set_relay(uint8_t status);
void send_primary_debug_1_signals(float field_1, float field_2, float field_3);
void send_primary_debug_2_signals(float field_1, float field_2, float field_3);
void send_primary_debug_3_signals(float field_1, float field_2, float field_3);
void send_primary_debug_4_signals(float field_1, float field_2, float field_3);
uint8_t get_health_status(void);

#endif  // BMS_LV_CONFIG_H
