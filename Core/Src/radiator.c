#include "radiator.h"

#include "bms_lv_config.h"
#include "pwm.h"
#include "tim.h"
#include <math.h>

float radiator_duty_cycle;
primary_lv_radiator_speed_status rad_status;

void radiator_init() {
  pwm_set_period(&RAD_HTIM, 0.04); // Set frequency to 25kHz
  pwm_set_duty_cicle(&RAD_HTIM, RAD_L_PWM_TIM_CHNL, 1.0);
  pwm_set_duty_cicle(&RAD_HTIM, RAD_R_PWM_TIM_CHNL, 1.0);
  pwm_start_channel(&RAD_HTIM, RAD_L_PWM_TIM_CHNL);
  pwm_start_channel(&RAD_HTIM, RAD_R_PWM_TIM_CHNL);
  radiator_duty_cycle = 0.0;
  rad_status = primary_lv_radiator_speed_status_off;
}

void radiator_set_duty_cycle(float duty_cycle) {
  float inverted_dt = 1.0 - duty_cycle;
  pwm_set_duty_cicle(&RAD_HTIM, RAD_L_PWM_TIM_CHNL, inverted_dt);
  pwm_set_duty_cicle(&RAD_HTIM, RAD_R_PWM_TIM_CHNL, inverted_dt);
  radiator_duty_cycle = duty_cycle;
}

float radiator_get_duty_cycle() { return radiator_duty_cycle; }

void radiator_set_status(primary_lv_radiator_speed_status status) {
  rad_status = status;
}

bool radiator_is_auto() {
  return rad_status == primary_lv_radiator_speed_status_auto;
}

primary_lv_radiator_speed_status radiator_get_status() { return rad_status; }

// TODO change duty cycles and temperature ranges
void radiator_auto_mode(float temp) {
  int8_t temp_rounded = (int8_t)round(temp);
  float duty_cycle = 1.0;
  switch (temp_rounded) {
  case INT8_MIN ... 49:
    duty_cycle = 0.0;
    break;
  case 50 ... 64:
    /* code */
    duty_cycle = 0.2;
    break;

  case 65 ... 74:
    /* code */
    duty_cycle = 0.5;
    break;

  case 75 ... INT8_MAX:
    duty_cycle = 1.0;
    break;

  default:
    break;
  }

  radiator_set_duty_cycle(duty_cycle);
}