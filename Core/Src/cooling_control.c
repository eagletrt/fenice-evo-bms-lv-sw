#include "cooling_control.h"

#if COOLING_TYPE == COOLING_TYPE_POLYNOMIAL

cooling_control_t current_cooling_control = cooling_control_weak;

static const float pumps_coefficients_weak[3]          = {0.06267081819754365f, -4.42371562013492f, 77.31101885486927f};
static const float radiator_coefficients_weak[3]       = {0.06860264519838995f, -5.620327774583103f, 108.79025301897668f};
static const float pumps_coefficients_normal[3]        = {0.12628672845592465f, -9.320252415697114f, 168.96963123644622f};
static const float radiator_coefficients_normal[3]     = {0.10048676874059574f, -8.098327285600545f, 157.13956987344554f};
static const float pumps_coefficients_aggressive[3]    = {0.2689248895434452f, -19.290132547864452f, 339.2268041237107f};
static const float radiator_coefficients_aggressive[3] = {0.169047619047619f, -11.964285714285769f, 208.16666666666936f};

void set_cooling_control(cooling_control_t cc) {
    current_cooling_control = cc;
}

float from_temperature_to_pumps_percentage(float temp) {
    if (temp < COOLING_OFF_THRESHOLD) {
        return 0.0f;
    }
    if (current_cooling_control == cooling_control_weak) {
        return ((pumps_coefficients_weak[0] * temp * temp) + (pumps_coefficients_weak[1] * temp) + pumps_coefficients_weak[2]) / 100.0f;
    } else if (current_cooling_control == cooling_control_normal) {
        return ((pumps_coefficients_normal[0] * temp * temp) + (pumps_coefficients_normal[1] * temp) + pumps_coefficients_normal[2]) /
               100.0f;
    } else {  // if (current_cooling_control == cooling_control_aggressive) {
        return ((pumps_coefficients_aggressive[0] * temp * temp) + (pumps_coefficients_aggressive[1] * temp) +
                pumps_coefficients_aggressive[2]) /
               100.0f;
    }
}

float from_temperature_to_radiator_percentage(float temp) {
    if (temp < COOLING_OFF_THRESHOLD) {
        return 0.0f;
    }
    if (current_cooling_control == cooling_control_weak) {
        return ((radiator_coefficients_weak[0] * temp * temp) + (radiator_coefficients_weak[1] * temp) + radiator_coefficients_weak[2]) /
               100.0f;
    } else if (current_cooling_control == cooling_control_normal) {
        return ((radiator_coefficients_normal[0] * temp * temp) + (radiator_coefficients_normal[1] * temp) +
                radiator_coefficients_normal[2]) /
               100.0f;
    } else {  // if (current_cooling_control == cooling_control_aggressive) {
        return ((radiator_coefficients_aggressive[0] * temp * temp) + (radiator_coefficients_aggressive[1] * temp) +
                radiator_coefficients_aggressive[2]) /
               100.0f;
    }
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
