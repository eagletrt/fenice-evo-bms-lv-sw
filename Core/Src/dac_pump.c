#include "dac_pump.h"

#include "bms_lv_config.h"
#include "cooling_control.h"
#include "dac.h"

#include <math.h>
#include <stdint.h>

float pump_duty_cycle;
primary_lv_pumps_speed_status pump_status;

void dac_pump_init() {
    pump_duty_cycle = 0.0;
    pump_status     = primary_lv_pumps_speed_status_off;

    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0U);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0U);
}

void dac_pump_set_duty_cycle(float duty_cycle) {
    float voltage        = 0.0;
    uint32_t analog_volt = 0.0;
    if (duty_cycle != 0.0) {
        voltage = duty_cycle * (MAX_OPAMP_OUT - MIN_OPAMP_OUT) + MIN_OPAMP_OUT;
    }
    analog_volt = (uint32_t)((voltage * MAX_DAC_OUT) / (MAX_OPAMP_OUT));

    HAL_DAC_SetValue(&PUMP_DAC, PUMP_L_CHNL, DAC_ALIGN_12B_R, analog_volt);
    HAL_DAC_SetValue(&PUMP_DAC, PUMP_R_CHNL, DAC_ALIGN_12B_R, analog_volt);

    pump_duty_cycle = duty_cycle;
}

float dac_pump_get_duty_cycle() {
    return pump_duty_cycle;
}

void dac_pump_set_status(primary_lv_pumps_speed_status status) {
    pump_status = status;
}

primary_lv_pumps_speed_status dac_pump_get_status() {
    return pump_status;
}

bool dac_pump_is_auto() {
    return pump_status == primary_lv_pumps_speed_status_auto;
}

void dac_pump_auto_mode(float temp) {
    float local_pumps_duty_cycle = from_temperature_to_pumps_percentage(temp);
    local_pumps_duty_cycle       = fminf(local_pumps_duty_cycle, 1.0f);
    local_pumps_duty_cycle       = fmaxf(local_pumps_duty_cycle, 0.0f);
    dac_pump_set_duty_cycle(local_pumps_duty_cycle);
}