#ifndef ERROR_SIMPLE_H
#define ERROR_SIMPLE_H

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define ERROR_SIMPLE_COUNTER_THRESHOLD (10U)
#define ERROR_SIMPLE_DUMP_SIZE         (50U)

typedef enum {
    ERROR_GROUP_BMS_LV_CELL_UNDERVOLTAGE,
    ERROR_GROUP_BMS_LV_CELL_OVERVOLTAGE,
    ERROR_GROUP_BMS_LV_OPEN_WIRE,
    ERROR_GROUP_BMS_LV_ADC,
    ERROR_GROUP_BMS_LV_CAN,
    ERROR_GROUP_BMS_LV_SPI,
    ERROR_GROUP_BMS_LV_OVER_CURRENT,
    ERROR_GROUP_BMS_LV_CELL_UNDER_TEMPERATURE,
    ERROR_GROUP_BMS_LV_CELL_OVER_TEMPERATURE,
    ERROR_GROUP_BMS_LV_MCP23017,
    ERROR_GROUP_BMS_LV_HEALTH,
    N_ERROR_GROUPS
} error_simple_groups_t;

typedef struct {
    error_simple_groups_t group;
    size_t instance;
} error_simple_dump_element_t;

#define ERROR_GROUP_BMS_LV_CELL_UNDERVOLTAGE_N_INSTANCES      (6U)
#define ERROR_GROUP_BMS_LV_CELL_OVERVOLTAGE_N_INSTANCES       (6U)
#define ERROR_GROUP_BMS_LV_OPEN_WIRE_N_INSTANCES              (1U)
#define ERROR_GROUP_BMS_LV_ADC_N_INSTANCES                    (1U)
#define ERROR_GROUP_BMS_LV_CAN_N_INSTANCES                    (2U)
#define ERROR_GROUP_BMS_LV_SPI_N_INSTANCES                    (1U)
#define ERROR_GROUP_BMS_LV_OVER_CURRENT_N_INSTANCES           (3U)
#define ERROR_GROUP_BMS_LV_CELL_UNDER_TEMPERATURE_N_INSTANCES (12U)
#define ERROR_GROUP_BMS_LV_CELL_OVER_TEMPERATURE_N_INSTANCES  (12U)
#define ERROR_GROUP_BMS_LV_MCP23017_N_INSTANCES               (1U)
#define ERROR_GROUP_BMS_LV_HEALTH_N_INSTANCES                 (1U)

#define ERROR_TOGGLE_IF(condition, group, instance) ((condition) ? error_simple_set(group, instance) : error_simple_reset(group, instance))

int error_simple_set(error_simple_groups_t group, size_t instance);
int error_simple_reset(error_simple_groups_t group, size_t instance);
int error_simple_routine(void);
size_t get_expired_errors(void);

extern error_simple_dump_element_t error_simple_dump[ERROR_SIMPLE_DUMP_SIZE];

size_t _error_simple_from_group_and_instance_to_index(error_simple_groups_t group, size_t instance);

#endif  // ERROR_SIMPLE_H
