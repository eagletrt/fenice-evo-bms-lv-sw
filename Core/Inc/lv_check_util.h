#ifndef LV_CHECK_UTIL_H
#define LV_CHECK_UTIL_H

#include <inttypes.h>
#include <stdbool.h>

#define NOT_FAULT_SCENARIOS_LENGTH 14
#define EDGE_CASES_LENGHT 4
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

typedef struct {
  uint8_t lvms_out : 1;  // 0 no voltage after LVMS, 1 voltage after LVMS
  uint8_t relay_out : 1; // 0 no voltage after Relay, 1 voltage after Relay
  uint8_t battery_voltage_out : 1; // 0 is above threshold, 1 is below threshold
                                   // - threshold: MIN_VOLTAGE
  uint8_t charger_current : 1;     // 0 is not charging, 1 is charging
  uint8_t battery_current : 1; // 0 is not flowing current, 1 is flowing current
  uint8_t sign_battery_current : 1; // O is negative, 1 is positive
} health_signal_t;

uint8_t not_fault_scenarios[NOT_FAULT_SCENARIOS_LENGTH] = {
    0b000010, 0b000100, 0b001011, 0b001110, 0b001111, 0b100010, 0b100100,
    0b101011, 0b101110, 0b101111, 0b011110, 0b011111, 0b110111, 0b111111};

int hs_check(float chg_current, float lvms_out, float relay_voltage,
             float batt_voltage, float batt_current);
#endif // LV_CHECK_UTIL_H