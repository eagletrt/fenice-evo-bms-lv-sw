#include "dac_pump.h"

#include "bms_lv_config.h"
#include "dac.h"
#include <math.h>
#include <stdint.h>

float pump_duty_cycle;
bool pump_auto_mode;

void dac_pump_init() {
  pump_duty_cycle = 0.0;
  pump_auto_mode = false;
}

void dac_pump_set_duty_cycle(float duty_cycle) {
  float voltage = 0.0;
  uint32_t analog_volt = 0.0;
  if (duty_cycle != 0.0) {
    voltage = duty_cycle * (MAX_OPAMP_OUT - MIN_OPAMP_OUT) + MIN_OPAMP_OUT;
  }
  analog_volt = (uint32_t)((voltage * MAX_DAC_OUT) / (MAX_OPAMP_OUT));

  HAL_DAC_Start(&PUMP_DAC, PUMP_L_CHNL);
  HAL_DAC_Start(&PUMP_DAC, PUMP_R_CHNL);
  HAL_DAC_SetValue(&PUMP_DAC, PUMP_L_CHNL, DAC_ALIGN_12B_R, analog_volt);
  HAL_DAC_SetValue(&PUMP_DAC, PUMP_L_CHNL, DAC_ALIGN_12B_R, analog_volt);

  pump_duty_cycle = duty_cycle;
}

float dac_pump_get_duty_cycle() { return pump_duty_cycle; }

void dac_pump_set_auto_mode(bool mode) { pump_auto_mode = mode; }

bool dac_pump_get_auto_mode() { return pump_auto_mode; }

// TODO change duty cycles and temperature ranges
void dac_pump_auto_mode(float temp) {
  int8_t temp_rounded = (int8_t)round(temp);
  float duty_cycle = 0.0;

  switch (temp_rounded) {
  case INT8_MIN ... 30:
    duty_cycle = 0.0;
    break;
  case 31 ... 45:
    /* code */
    duty_cycle = 0.5;
    break;

  case 46 ... 60:
    /* code */
    duty_cycle = 0.75;
    break;

  case 61 ... INT8_MAX:
    duty_cycle = 1.0;
    break;

  default:
    break;
  }

  dac_pump_set_duty_cycle(duty_cycle);
}
