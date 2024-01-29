#ifndef LV_ERRORS_H
#define LV_ERRORS_H

#include "error_utils.h"

#define LV_ERROR_BUFFER_SIZE 100
#define TIMEOUT_NEVER __UINT32_MAX__

#define LV_ERROR_SET_INT(error, instance)                                      \
  ERROR_UTILS_SET_INT(&error_handler, error, instance)
#define LV_ERROR_SET_STR(error, instance)                                      \
  ERROR_UTILS_SET_STR(&error_handler, error, instance)

#define LV_ERROR_RESET_INT(error, instance)                                    \
  ERROR_UTILS_RESET_INT(&error_handler, error, instance)
#define LV_ERROR_RESET_STR(error, instance)                                    \
  ERROR_UTILS_RESET_STR(&error_handler, error, instance)

#define LV_ERROR_TOGGLE_CHECK_INT(condition, error, instance)                  \
  if ((condition)) {                                                           \
    LV_ERROR_SET_INT(error, instance);                                         \
  } else {                                                                     \
    LV_ERROR_RESET_INT(error, instance);                                       \
  }

#define LV_ERROR_TOGGLE_CHECK_STR(condition, error, instance)                  \
  if ((condition)) {                                                           \
    LV_ERROR_SET_STR(error, instance);                                         \
  } else {                                                                     \
    LV_ERROR_RESET_STR(error, instance);                                       \
  }

typedef enum {
  ERROR_CELL_UNDERVOLTAGE,
  ERROR_CELL_OVERVOLTAGE,
  ERROR_OPEN_WIRE,
  ERROR_CAN,
  ERROR_SPI,
  ERROR_OVER_CURRENT,
  ERROR_CELL_UNDER_TEMPERATURE,
  ERROR_CELL_OVER_TEMPERATURE,
  ERROR_RELAY,
  ERROR_BMS_MONITOR,
  ERROR_VOLTAGES_NOT_READY,
  ERROR_MCP23017,
  ERROR_RADIATOR,
  ERROR_FAN,
  ERROR_PUMP,
  ERROR_ADC_INIT,
  ERROR_ADC_MUX,
  ERRROR_LV_SUPPLY,
  ERROR_NUM_ERRORS

} __attribute__((__packed__)) error_id;

extern ErrorUtilsHandler error_handler;

void lv_error_init();

size_t lv_error_running_count();

size_t lv_error_expire_count();

#endif // LV_ERRORS_H