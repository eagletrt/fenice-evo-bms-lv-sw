#ifndef DAC_PUMP_H
#define DAC_PUMP_H

#include "primary_network.h"

#include <stdbool.h>

#define MAX_DAC_OUT       4096.0f
#define MAX_GPIO_OUT      3.3
#define AMPLIFICATOR_GAIN 1.5
#define MAX_OPAMP_OUT     MAX_GPIO_OUT *AMPLIFICATOR_GAIN
#define MIN_OPAMP_OUT     2.0

void dac_pump_init();
void dac_pump_set_duty_cycle(float duty_cycle);
float dac_pump_get_duty_cycle();
void dac_pump_set_status(primary_lv_pumps_speed_status mode);
primary_lv_pumps_speed_status dac_pump_get_status();
bool dac_pump_is_auto();
void dac_pump_auto_mode(float temp);

#endif  // DAC_PUMP_H