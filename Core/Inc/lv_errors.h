/*******************************************************************************
 * Critical error handler library generator
 * Generated by error_gen ruby gem, for more information see:
 * https://github.com/eagletrt/micro-utils/tree/master/error-handler-generator
 *
 * Error_gen version 1.3.3
 * Generation date: 2024-04-17 14:13:56 +0200
 * Generated from: lv_errors.json
 * The error handler contains:
 *     - 11 error groups
 *     - 46 total error instances
 ******************************************************************************/

#ifndef LV_ERRORS_H
#define LV_ERRORS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Total number of error instances
#define ERROR_INSTANCE_COUNT 46

/**
 * @brief Set or reset an instance of an error based on a condition
 * @details If the condition is true the error is set, otherwise it is reset
 *
 * @param condition A boolean expression
 * @param group The group to which the error belongs
 * @param instance The instance of the error
 * @param The current time (in ms)
 */
#define ERROR_TOGGLE_IF(condition, group, instance, timestamp) \
    ((condition) ? error_set(group, instance, timestamp) : error_reset(group, instance))

/** @brief Type of the error that categorize a group of instances */
typedef enum {
    BMS_LV_CELL_UNDERVOLTAGE,
    BMS_LV_CELL_OVERVOLTAGE,
    BMS_LV_OPEN_WIRE,
    BMS_LV_ADC,
    BMS_LV_CAN,
    BMS_LV_SPI,
    BMS_LV_OVER_CURRENT,
    BMS_LV_CELL_UNDER_TEMPERATURE,
    BMS_LV_CELL_OVER_TEMPERATURE,
    BMS_LV_MCP23017,
    BMS_LV_HEALTH,
    ERROR_COUNT
} ErrorGroup;

// Single error instance type definition
typedef uint16_t ErrorInstance;

/**
 * @brief Error type definition
 *
 * @param group The group to which the error belongs
 * @param timestamp The time when the error was set (in ms)
 * @param is_running True if the error is set, false otherwise
 * @param is_expired True if the error has expired, false otherwise
 */
typedef struct {
    ErrorGroup group;
    uint32_t timestamp;
    bool is_running;
    bool is_expired;
} Error;

/**
 * @brief Initialize the internal error handler structures
 * @details A critical section is defined as a block of code where, if an interrupt
 * happens, undefined behaviour with the modified data within the block can happen
 *
 * @param cs_enter A pointer to a function that should manage a critical section
 * @param cs_exit A pointer to a function that shuold manage a critical section
 */
void error_init(void (*cs_enter)(void), void (*cs_exit)(void));

/**
 * @brief Get the number of errors that has been set but they still have to expire
 * 
 * @param size_t The number of running errors
 */
size_t error_get_running(void);

/**
 * @brief Get the number of expired errors
 * 
 * @param size_t The number of expired errors
 */
size_t error_get_expired(void);

/**
 * @brief Get the number of running error of a specific group
 *
 * @param group The error group
 * @return uint16_t The number of running errors
 */
uint16_t error_get_group_running(ErrorGroup group);

/**
 * @brief Get the number of expired error of a specific group
 *
 * @param group The error group
 * @return uint16_t The number of running errors
 */
uint16_t error_get_group_expired(ErrorGroup group);

/**
 * @brief Get a copy of all the errors that are currently running
 * @attention This function can be quite expensive in terms of time
 * and should be used wisely, do not call to often
 * @attention This function calls the critical section handler functions
 * @details The out array should be able to contain all the instances
 *
 * @param out A pointer to an array of errors where the data is copied into
 * @return size_t The number of copied errors
 */
size_t error_dump_running(Error *out);

/**
 * @brief Get a copy of all the errors that are expired
 * @attention This function can be quite expensive in terms of time
 * and should be used wisely, do not call to often
 * @attention This function calls the critical section handler functions
 * @details The out array should be able to contain all the instances
 *
 * @param out A pointer to an array of errors where the data is copied into
 * @return size_t The number of copied errors
 */
size_t error_dump_expired(Error *out);

/**
 * @brief Get all the groups in which at least one error is running
 * 
 * @param out A pointer to an array of groups where the data is copied into
 * @return size_t The number of copied groups
 */
size_t error_dump_running_groups(ErrorGroup *out);

/**
 * @brief Get all the groups in which at least one error is expired
 * 
 * @param out A pointer to an array of groups where the data is copied into
 * @return size_t The number of copied groups
 */
size_t error_dump_expired_groups(ErrorGroup *out);

/**
 * @brief Set an error which will expire after a certain amount of time (the timeout)
 * 
 * @param group The group to which the error belongs
 * @param instance The instance of the error
 * @param The current time (in ms)
 */
void error_set(ErrorGroup group, ErrorInstance instance, uint32_t timestamp);

/**
 * @brief Reset an error to avoid its expiration
 *
 * @param group The group to which the error belongs
 * @param instance The instance of the error
 */
void error_reset(ErrorGroup group, ErrorInstance instance);

/** @brief Set the error as expired */
void error_expire(void);

/**
 * @brief Routine that updates the internal error states
 * @attention This function should not be called inside interrupts callback
 * or other threads
 * @details This function should be called periodically
 */
void error_routine(void);

/**
 * @brief Update the timer that should expire the error after a certain amount of time
 * @attention This function have to be defined by the user
 * @details This function is called internally when an error is set, reset or expired
 *
 * @param timestamp The time in which the error was set (in ms)
 * @param timeout The time that should elapse after the timestamp to expire the error (in ms)
 */
void error_update_timer_callback(uint32_t timestamp, uint16_t timeout);

/**
 * @brief Stop the timer that should expire the errors
 * @attention This function have to be defined by the user
 * @details This function is called internally when an error is reset or expired
 */
void error_stop_timer_callback(void);

#endif  // LV_ERRORS_H
