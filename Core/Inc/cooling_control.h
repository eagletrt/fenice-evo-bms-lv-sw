#ifndef COOLING_CONTROL_H
#define COOLING_CONTROL_H

#include "bms_lv_config.h"

#if COOLING_TYPE == COOLING_TYPE_POLYNOMIAL

#define COOLING_OFF_THRESHOLD (40.0f)

typedef enum { cooling_control_weak = 0, cooling_control_normal, cooling_control_aggressive } cooling_control_t;

void set_cooling_control(cooling_control_t cc);

#elif COOLING_TYPE == COOLING_TYPE_PID

#define COOLING_PID_PUMPS_EFFORT     (0.8f)
#define COOLING_PID_RADIATORS_EFFORT (0.2f)

void cooling_pid_init(float kp, float ki, float kd, float sample_time, float anti_windUp);

#endif  // COOLING_TYPE

float from_temperature_to_pumps_percentage(float);
float from_temperature_to_radiator_percentage(float);

#endif  // COOLING_CONTROL_H
