/**
 * @file      current_transducer.h
 * @author    Simone Ruffini [simone.ruffini@tutanota.com]
 * @author    Giacomo Mazzucchi [giacomo.mazzucchi@protonmail.com]
 * @date      Sun Jan 28 09:42:42 PM CET 2024
 *
 * @prefix    CT
 *
 * @brief     Current Transducer Api
 *
 */

#ifndef __CURRENT_TRANSDUCER_H__
#define __CURRENT_TRANSDUCER_H__

#include "bms_lv_config.h"

/**
 * @brief Theoretical sensitivity expressed in mV/A
 * Sensitivity: If in the transducer flows 1A of current then the output
 * voltage vould be Vout=Vref+(1A*Sensitivity)
 */
#define HO_50_SP33_1106_THEORETICAL_SENSITIVITY (9.2f)

/**
 * @brief Voltage reference of the current tranducer
 * This value should not be static because the LEM HO 50S has an internal
 * reference calculated from the VDD. The internal reference can be read from
 * the 4th pin (Vref) and used to adjust the reading, but at the current time
 * this output is not wired in to a MCU pin. Therefore we assume the optimal
 * value given Vdd = 3.3V (PROJECT_FOLDER/Doc/ho-s_sp33-1106_series.pdf page 4)
 */
#define HO_50_SP33_1106_VREF_mV (1645U)

/**
 * @brief OverCurrentDetection multiplier value:
 *  OCD is on when the current flowing the current transducer is
 *  OCD_MULT*Ipn  (Ipn = primary nominal current)
 *
 */
#define HO_50_SP33_1106_OCD_MULT (2.92f)

/**
 * @brief  Get electric current value currently flowing in the Current
 * Transducer
 * @param adc_raw_value ADC output already calibrated in mV
 * @param sensor_vref Voltage of sensor at 0 Amps
 * @return Current in mA flowing inside the transducer
 *         if 0xFFFF then error
 */
#define CT_get_electric_current_mA(adc_value_mV) \
    (((adc_value_mV - HO_50_SP33_1106_VREF_mV) / HO_50_SP33_1106_THEORETICAL_SENSITIVITY) * 1000)

#endif
