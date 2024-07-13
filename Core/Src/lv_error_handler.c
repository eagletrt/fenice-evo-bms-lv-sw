#include "bms_lv_config.h"
#include "lv_errors.h"
#include "tim.h"
#include "timer_utils.h"

uint32_t primask;
void error_cs_enter(void) {
    primask = __get_PRIMASK();
    __disable_irq();
}
void error_cs_exit(void) {
    if (!primask)
        __enable_irq();
}

void lv_error_init(void) {
    error_init(error_cs_enter, error_cs_exit);
}

void error_update_timer_callback(uint32_t timestamp, uint16_t timeout) {
    HAL_TIM_Base_Stop_IT(&ERROR_TIMER);

    int32_t t  = HAL_GetTick();
    int32_t dt = ((int32_t)timestamp - t) + (int32_t)timeout;
    if (dt <= 0)
        dt = 0;
    __HAL_TIM_SET_COUNTER(&ERROR_TIMER, 0);
    __HAL_TIM_SET_AUTORELOAD(&ERROR_TIMER, TIM_MS_TO_TICKS(&ERROR_TIMER, dt));
    __HAL_TIM_CLEAR_FLAG(&ERROR_TIMER, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&ERROR_TIMER);
}
void error_stop_timer_callback(void) {
    HAL_TIM_Base_Stop_IT(&ERROR_TIMER);
}

bool error_get_fatal(void) {
    return error_get_expired() > 0;
}
