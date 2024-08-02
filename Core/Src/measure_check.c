#include "bms_lv_config.h"
#include "lv_errors.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

void set_relay(uint8_t status);
void buzzer_beep_async(uint32_t buzzer_duration, buzzer_mode_t sound_mode);

extern float mux_fb_mV[mux_fb_n_values];
extern float mux_sensors_mA[mux_sensors_n_values];
extern float dc_fb_mV[directly_connected_fbs_n_values];
extern uint8_t mcp23017_feedbacks_state[8];
uint8_t health_status = 0;

/* Due to hardware problems (maybe) of the LTC and the electric circuit, voltage readings during charging are sometimes incorrect */
bool disable_voltage_checks = false;

// Index is from right to left
void set_bit(uint8_t *value, uint8_t index, uint8_t bit_value) {
    if (bit_value == 0) {
        *value = *value & ~(1 << index);
    } else if (bit_value == 1) {
        *value = *value | (1 << index);
    }
}

uint8_t convert_relay_out_ang_to_state(int relay_out, int bat_out) {
    return (
        (abs(relay_out - bat_out) < MIN_RELAY_VOLTAGE_DIFF_THRESHOLD_mV) && (relay_out > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV) &&
        (bat_out > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV));
}

uint8_t convert_lvms_out_ang_to_state(int relay_out, int lvms_out) {
    return (
        (abs(relay_out - lvms_out) < MIN_LVMS_VOLTAGE_DIFF_THRESHOLD_mV) && (relay_out > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV) &&
        (lvms_out > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV));
}

uint8_t get_health_status(float i_bat, float i_chg, int bat_out, int relay_out, int lvms_out) {
    uint8_t current_status = 0;
    set_bit(&current_status, (HEALTH_SIGNALS_N - HEALTH_IBAT_SIGN_INDEX - 1), i_bat < 0 ? 0 : 1);
    set_bit(&current_status, (HEALTH_SIGNALS_N - HEALTH_IBAT_INDEX - 1), i_bat < MIN_BATTERY_CURRENT_THRESHOLD_mA ? 0 : 1);
    set_bit(&current_status, (HEALTH_SIGNALS_N - HEALTH_ICHARGER_INDEX - 1), i_chg < MIN_CHARGER_CURRENT_THRESHOLD_mA ? 0 : 1);
    set_bit(&current_status, (HEALTH_SIGNALS_N - HEALTH_BATTOUT_INDEX - 1), ((float)(bat_out)) < MIN_BATTERY_VOLTAGE_mV ? 0 : 1);
    set_bit(&current_status, (HEALTH_SIGNALS_N - HEALTH_RELAY_OUT_INDEX - 1), convert_relay_out_ang_to_state(relay_out, bat_out));
    set_bit(&current_status, (HEALTH_SIGNALS_N - HEALTH_LVMS_OUT_INDEX - 1), convert_lvms_out_ang_to_state(relay_out, lvms_out));
    return current_status;
}

int check_status(uint8_t current_status) {
    switch (current_status) {
        case HEALTH_SAFE_STATUS_CHG_CONNECT_ONLY_MCU_PW:
        case HEALTH_SAFE_STATUS_CAR_ON_CHG_ONLY:
            set_relay(0);
            break;
        case HEALTH_SAFE_STATUS_MCU_ON_BAT_ONLY:
        case HEALTH_SAFE_STATUS_CHG_BATT:
        case HEALTH_SAFE_STATUS_CAR_ON_BAT_ONLY:
        case HEALTH_SAFE_STATUS_RELAY_CLOSED_CHG_CAR_KEEP_CLOSED:
        case HEALTH_SAFE_STATUS_CAR_ON_CHG_AND_BAT:
        case HEALTH_SAFE_STATUS_CAR_RUNNING:
        case HEALTH_SAFE_STATUS_CAR_ON_CHG_AND_BAT_DUPLICATED:
            break;

        default:
            return HEALTH_ERROR_STATUS;
            break;
    }

    return HEALTH_SAFE_STATUS;
}

uint32_t last_charging_time = 0;
bool lv_is_charging         = false;

bool disable_undervoltage_and_overvoltage_checks(float i_chg) {
    if (i_chg > MIN_CHARGER_CURRENT_THRESHOLD_mA) {
        lv_is_charging     = true;
        last_charging_time = get_current_time_ms();
    }
    if (lv_is_charging && ((get_current_time_ms() - last_charging_time) > LV_VOLTAGE_CHECKS_DISABLED_TIMEOUT_ms)) {
        lv_is_charging = false;
    }
    return lv_is_charging;
}

void health_check(void) {
    float i_bat, i_chg = 0.0f;
    int bat_out, relay_out, lvms_out = 0;
    uint8_t check_result = HEALTH_ERROR_STATUS;
    i_bat                = mux_sensors_mA[mux_sensors_s_hall1_idx];
    i_chg                = mux_sensors_mA[mux_sensors_s_hall2_idx];
    bat_out              = (int)dc_fb_mV[fb_batt_out_idx];
    relay_out            = (int)dc_fb_mV[fb_relay_out_idx];
    lvms_out             = (int)dc_fb_mV[fb_lvms_out_idx];

    disable_voltage_checks = disable_undervoltage_and_overvoltage_checks(i_chg);

    health_status = get_health_status(i_bat, i_chg, bat_out, relay_out, lvms_out);
    check_result  = check_status(health_status);
    if (check_result != HEALTH_SAFE_STATUS && get_current_time_ms() > 2000) {
        error_set(ERROR_GROUP_BMS_LV_HEALTH, 0, get_current_time_ms());
    } else {
        error_reset(ERROR_GROUP_BMS_LV_HEALTH, 0);
    }
}

void single_cells_voltages_checks(void) {
    float voltages[CELL_COUNT] = {0};
    monitor_get_voltages(voltages);
    float min_voltage = FLT_MAX;

    if (disable_voltage_checks) {
        /* TODO Undevoltage and overvoltage protection even while charging and voltage readings are incorrect */
    } else {
        for (size_t i = 0; i < CELL_COUNT; i++) {
            if (voltages[i] < MIN_CELL_VOLTAGE_V) {
                error_set(ERROR_GROUP_BMS_LV_CELL_UNDERVOLTAGE, i, get_current_time_ms());
            }
            if (voltages[i] > MAX_CELL_VOLTAGE_V) {
                error_set(ERROR_GROUP_BMS_LV_CELL_OVERVOLTAGE, i, get_current_time_ms());
            }
            min_voltage = fmin(min_voltage, voltages[i]);
        }

        static uint32_t last_undervoltage_warning = 0;
        if ((min_voltage < WARNING_VOLTAGE_V) && ((get_current_time_ms() - last_undervoltage_warning) > BUZZER_WARNING_INTERVAL)) {
            last_undervoltage_warning = get_current_time_ms();
            buzzer_beep_async(WARNING_BUZZ_DURATION_ms, BUZZER_MODE_WARNING);
        }

        for (size_t i = 0; i < CELL_COUNT; i++) {
            if (voltages[i] >= MIN_CELL_VOLTAGE_V) {
                error_reset(ERROR_GROUP_BMS_LV_CELL_UNDERVOLTAGE, i);
            }
            if (voltages[i] <= MAX_CELL_VOLTAGE_V) {
                error_reset(ERROR_GROUP_BMS_LV_CELL_OVERVOLTAGE, i);
            }
        }
    }
}

bool is_total_voltage_ok(void) {
    bool total_voltage_ok      = false;
    float voltages[CELL_COUNT] = {0};
    float total_voltage        = 0.0f;

    monitor_get_voltages(voltages);
    total_voltage = (voltages[0] + voltages[1] + voltages[2] + voltages[3] + voltages[4] + voltages[5]) * 1000.0f;
    if (total_voltage >= MIN_BATTERY_VOLTAGE_mV) {
        total_voltage_ok = true;
    }

    return total_voltage_ok;
}

void cell_temperature_check(void) {
    float temperatures[TEMP_SENSOR_COUNT] = {0};
    monitor_get_temperatures(temperatures);

    for (size_t i = 0; i < TEMP_SENSOR_COUNT; i++) {
        ERROR_TOGGLE_IF(temperatures[i] < MIN_CELL_TEMP, ERROR_GROUP_BMS_LV_CELL_UNDER_TEMPERATURE, i, get_current_time_ms());
        ERROR_TOGGLE_IF(temperatures[i] > MAX_CELL_TEMP, ERROR_GROUP_BMS_LV_CELL_OVER_TEMPERATURE, i, get_current_time_ms());
    }
}

void overcurrent_check(void) {
    ERROR_TOGGLE_IF(mux_sensors_mA[mux_sensors_s_hall1_idx] > MAX_CURRENT_mA, ERROR_GROUP_BMS_LV_OVER_CURRENT, 1, get_current_time_ms());
    ERROR_TOGGLE_IF(mux_sensors_mA[mux_sensors_s_hall2_idx] > MAX_CURRENT_mA, ERROR_GROUP_BMS_LV_OVER_CURRENT, 2, get_current_time_ms());
}

void all_measurements_check(void) {
    health_check();
    single_cells_voltages_checks();
    cell_temperature_check();
    overcurrent_check();
}
