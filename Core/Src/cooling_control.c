#include "cooling_control.h"

#if COOLING_TYPE == COOLING_TYPE_POLYNOMIAL

static double pumps_coeffs[3]     = PUMPS_COEFFICIENTS;
static double radiators_coeffs[3] = RADIATOR_COEFFICIENTS;

float from_temperature_to_pumps_percentage(float temp) {
    if (temp < COOLING_OFF_THRESHOLD) {
        return 0.0f;
    }
    return ((pumps_coeffs[0] * temp * temp) + (pumps_coeffs[1] * temp) + pumps_coeffs[2]) / 100.0f;
}

float from_temperature_to_radiator_percentage(float temp) {
    if (temp < COOLING_OFF_THRESHOLD) {
        return 0.0f;
    }
    return ((radiators_coeffs[0] * temp * temp) + (radiators_coeffs[1] * temp) + radiators_coeffs[2]) / 100.0f;
}

#elif COOLING_TYPE == COOLING_TYPE_PID

#include "pid.h"

#include <assert.h>

static PidController_t cooling_pid;

void _compile_time_checks(void) {
    static_assert((COOLING_PID_PUMPS_EFFORT + COOLING_PID_RADIATORS_EFFORT) == 1.0f);
}

void cooling_pid_init(float kp, float ki, float kd, float sample_time, float anti_windUp) {
    pid_init(&cooling_pid, kp, ki, kd, sample_time, anti_windUp);
}

float from_temperature_to_pumps_percentage(float temp) {
    pid_update(&cooling_pid, temp);
    return pid_compute(&cooling_pid) * COOLING_PID_PUMPS_EFFORT;
}

float from_temperature_to_radiator_percentage(float temp) {
    pid_update(&cooling_pid, temp);
    return pid_compute(&cooling_pid) * COOLING_PID_RADIATORS_EFFORT;
}

#endif  // COOLING_TYPE
