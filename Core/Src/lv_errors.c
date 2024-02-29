#include "lv_errors.h"

#if 0
#include <math.h>

ErrorUtilsHandler error_handler;
ErrorUtilsRunningInstance lv_errors[LV_ERROR_BUFFER_SIZE];
ErrorUtilsRunningInstance *lv_errors_ptr[LV_ERROR_BUFFER_SIZE];
// uint32_t rPriMask = 0;

const uint32_t lv_error_timeout[ERROR_NUM_ERRORS] = {
    [ERROR_CELL_UNDERVOLTAGE] = 400,
    [ERROR_CELL_OVERVOLTAGE] = 400,
    [ERROR_OPEN_WIRE] = 400,
    [ERROR_CAN] = 500,
    [ERROR_SPI] = 500,
    [ERROR_OVER_CURRENT] = 400,
    [ERROR_CELL_UNDER_TEMPERATURE] = 1000,
    [ERROR_CELL_OVER_TEMPERATURE] = 200,
    [ERROR_RELAY] = TIMEOUT_NEVER,
    [ERROR_BMS_MONITOR] = 500,
    [ERROR_VOLTAGES_NOT_READY] = 500,
    [ERROR_MCP23017] = 10000,
    [ERROR_RADIATOR] = TIMEOUT_NEVER,
    [ERROR_FAN] = TIMEOUT_NEVER,
    [ERROR_PUMP] = TIMEOUT_NEVER,
    [ERROR_ADC_INIT] = 1000,
    [ERROR_ADC_MUX] = 500,
    [ERRROR_LV_SUPPLY] = 500,
};

uint32_t lv_get_timestamp();
uint32_t lv_error_get_timeout(uint32_t error);
void lv_error_set_expire(uint32_t timestamp, uint32_t timeout);
void lv_error_expire_stop();
void cs_enter();
void cs_exit();

void lv_error_init() {
  error_utils_init(&error_handler, lv_errors, lv_errors_ptr,
                   LV_ERROR_BUFFER_SIZE, lv_get_timestamp, lv_error_get_timeout,
                   lv_error_set_expire, lv_error_expire_stop, cs_enter,
                   cs_exit);
}

uint32_t lv_get_timestamp() { return HAL_GetTick(); }

uint32_t lv_error_get_timeout(uint32_t error) {
  return lv_error_timeout[error];
}

void lv_error_set_expire(uint32_t timestamp, uint32_t timeout) {
  if (timeout == TIMEOUT_NEVER)
    return;

  lv_error_expire_stop();

  int32_t delta = (int32_t)(timestamp + timeout) - (int32_t)HAL_GetTick();
  uint16_t cnt = __HAL_TIM_GET_COUNTER(&HTIM_ERR);
  __HAL_TIM_SET_COMPARE(&HTIM_ERR, TIM_CHANNEL_1,
                        cnt + TIM_MS_TO_TICKS(&HTIM_ERR, delta));

  HAL_TIM_OC_Start_IT(&HTIM_ERR, TIM_CHANNEL_1);
}

void lv_error_expire_stop() {
  // hai resettato l'ultimo errore che c'era -> bom puoi levare il timer

  HAL_TIM_OC_Stop_IT(&HTIM_ERR, TIM_CHANNEL_1);
  __HAL_TIM_CLEAR_FLAG(&HTIM_ERR, TIM_IT_CC1);
}

void cs_enter() {
  // TODO: the library should not disable the interrupts
  // rPriMask = __get_PRIMASK();
  // __set_PRIMASK(1);
}

void cs_exit() {
  // TODO: the library should not disable the interrupts
  // __set_PRIMASK(rPriMask);
}

size_t lv_error_running_count() {
  return error_utils_running_count(&error_handler);
}

size_t lv_error_expire_count() {
  return error_utils_expired_count(&error_handler);
}

// HEALTH SIGNALS

#define NOT_FAULT_SCENARIOS_LENGTH 14
// #define EDGE_CASES_LENGHT 4
#define MIN_CHARGER_CURRENT_THRESHOLD_mA (4000.0f)
#define MIN_BATTERY_CURRENT_THRESHOLD_mA (50.0f)
#define MIN_BATTERY_VOLTAGE_mV (3300.0 * 6.0f)
// Min difference threshold between V Relay and V Battery
#define MIN_RELAY_VOLTAGE_DIFF_THRESHOLD_mV                                    \
  (2000.0f) // diff v relay (that could be the charger one and bat out)
// Min difference threshold between LVMS out V ans V Relay
#define MIN_LVMS_VOLTAGE_DIFF_THRESHOLD_mV                                     \
  (2000.0f) // diff lvms out and relay out 5%
#define MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV (500.0f) // 500 mV

// TODO: check and double check these values
uint8_t not_fault_scenarios[NOT_FAULT_SCENARIOS_LENGTH] = {
    0b000010, 0b000100, 0b001011, 0b001110, 0b001111, 0b100010, 0b100100,
    0b101011, 0b101110, 0b101111, 0b011110, 0b011111, 0b110111, 0b111111};

int hs_check(float chg_current, float lvms_out, float relay_voltage,
             float batt_voltage, float batt_current) {
  uint8_t health_signal_current_value = 0;
  float voltage_diff = fabs(relay_voltage - lvms_out);

  if (voltage_diff < MIN_LVMS_VOLTAGE_DIFF_THRESHOLD_mV &&
      relay_voltage > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV &&
      lvms_out > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV) {
    health_signal_current_value |= 1;
  }
  voltage_diff = fabs(relay_voltage - batt_voltage);
  if (voltage_diff < MIN_RELAY_VOLTAGE_DIFF_THRESHOLD_mV &&
      relay_voltage > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV &&
      batt_voltage > MIN_LOW_LOGIC_LEVEL_THRESHOLD_mV) {
    health_signal_current_value |= 1 << 1;
  }
  if (batt_voltage >= MIN_BATTERY_VOLTAGE_mV) {
    health_signal_current_value |= 1 << 2;
  }
  if (chg_current >= MIN_CHARGER_CURRENT_THRESHOLD_mA) {
    health_signal_current_value |= 1 << 3;
  }
  if (batt_current >= MIN_BATTERY_CURRENT_THRESHOLD_mA) {
    health_signal_current_value |= 1 << 4;
  }
  if (batt_current >= 0) {
    health_signal_current_value |= 1 << 5;
  }
  int fault = 1;
  for (size_t infs = 0; infs < NOT_FAULT_SCENARIOS_LENGTH && fault; infs++) {
    if (not_fault_scenarios[infs] == health_signal_current_value) {
      fault = 0;
    }
  }
  return fault;
}
#endif
