#include "cooling_control.h"

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
