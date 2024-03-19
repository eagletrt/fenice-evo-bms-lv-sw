#include "radiator.h"

#include "bms_lv_config.h"
#include "pwm.h"
#include "tim.h"

float radiator_duty_cycle;
bool rad_auto_mode;

void radiator_init() {
  pwm_set_period(&RAD_HTIM, 0.04); // Set frequency to 25kHz
  pwm_set_duty_cicle(&RAD_HTIM, RAD_L_PWM_TIM_CHNL, 1.0);
  pwm_set_duty_cicle(&RAD_HTIM, RAD_R_PWM_TIM_CHNL, 1.0);
  pwm_start_channel(&RAD_HTIM, RAD_L_PWM_TIM_CHNL);
  pwm_start_channel(&RAD_HTIM, RAD_R_PWM_TIM_CHNL);
  radiator_duty_cycle = 0.0;
  rad_auto_mode = false;
}

void radiator_set_duty_cycle(float duty_cycle) {
  float inverted_dt = 1.0 - duty_cycle;
  pwm_set_duty_cicle(&RAD_HTIM, RAD_L_PWM_TIM_CHNL, inverted_dt);
  pwm_set_duty_cicle(&RAD_HTIM, RAD_R_PWM_TIM_CHNL, inverted_dt);
}

float get_radiator_duty_cycle() { return radiator_duty_cycle; }

void radiator_set_auto_mode(bool mode) { rad_auto_mode = mode; }

bool radiator_get_auto_mode() { return rad_auto_mode; }

// TODO change duty cycles and temperature ranges
void radiator_auto_mode(float temp) {
  int8_t temp_rounded = (int8_t)round(temp);
  float duty_cycle = 1.0;
  switch (temp_rounded) {
  case INT8_MIN ... 30:
    duty_cycle = 1.0;
    break;
  case 31 ... 45:
    /* code */
    duty_cycle = 0.5;
    break;

  case 46 ... 60:
    /* code */
    duty_cycle = 0.25;
    break;

  case 61 ... INT8_MAX:
    duty_cycle = 0.0;
    break;

  default:
    break;
  }

  radiator_set_duty_cycle(duty_cycle);
}