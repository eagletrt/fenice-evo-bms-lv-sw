/**
 * @file      current_sensor.h
 * @author    Simone Ruffini [simone.ruffini@tutanota.com]
 * @date      Fri May 20 05:35:10 PM CEST 2022
 * 
 * @prefix    CT
 * 
 * @brief     Current Transducer Api
 * 
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/**
 * @brief overcurrent threshold in milliAmpere
 */
#define CT_OVERCURRENT_THRESHOLD_MA (50000U)

/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief Return if an overcurrent is detected
 * 
 * @return true 
 * @return false 
 */
bool CS_is_overcurrent();

/**
 * @brief  Get electric current value currently flowing in the Current Transducer
 * @param adc_raw_value ADC output
 * @param sensor_vref Voltage of sensor at 0 Amps
 * @return Current in mA flowing inside the transducer
 *         if 0xFFFF then error
 */
float CS_get_electric_current_mA(uint32_t adc_raw_value);

/**
 * @brief Over-Current Detection callback (put this in the correct ISR)
 * 
 * @param isOn value of the OCD pin: true if rising edge false if falling edge
 *             of OCD pin
 */
void CS_OCD_callback(bool isOn);

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private Macros -----------------------------------------------------------*/
#endif  //CURRENT_SENSOR_H
