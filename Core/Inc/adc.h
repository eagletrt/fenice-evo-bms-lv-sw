/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN Private defines */

//Hardware
#define ADC_HALL_AND_FB                 &hadc2
#define ADC_VREF_CALIBRATION            &hadc1
#define TIMER_ADC_CALIBRATION           &htim1
#define TIMER_ADC_ROUTINE               &htim6
#define TIMER_ADC_CALIBRATION_CHANNEL   TIM_CHANNEL_1
//Average history
#define HISTORY_L                       25
#define ADC2_CHANNELS_N                 6
#define MUX_CHANNELS_N                  16
#define ADC_CALIBRATION_CHANNELS_N      2
#define ADC_CALIBRATION_SAMPLES_N       500
//(multiplexer channels * 2) + 4 adc2_channels
#define MEAN_VALUES_ARRAY_LEN           ((MUX_CHANNELS_N * 2) + 4)
#define S_HALL0                         1
#define S_HALL1                         3
#define S_HALL2                         5
#define S_HALL1_OFFSET_mA               300.0f
#define S_HALL2_OFFSET_mA               1500.0f
#define ADC2_VOLTAGE_DIVIDER_MULTIPLIER 9.0f

bool start_dma_read;
bool start_value_conversion;
bool start_calculating_averages;

enum multiplexer_channel_addresses {
  multiplexer_channel_0,
  multiplexer_channel_8,
  multiplexer_channel_4,
  multiplexer_channel_12,
  multiplexer_channel_2,
  multiplexer_channel_10,
  multiplexer_channel_6,
  multiplexer_channel_14,
  multiplexer_channel_1,
  multiplexer_channel_9,
  multiplexer_channel_5,
  multiplexer_channel_13,
  multiplexer_channel_3,
  multiplexer_channel_11,
  multiplexer_channel_7,
  multiplexer_channel_15
};

enum ADC2_channels {
  adc2_channel_mux_hall,
  adc2_channel_mux_fb,
  adc2_channel_adcs_as_computer_fb,
  adc2_channel_adcs_relay_out,
  adc2_channel_adcs_lvms_out,
  adc2_channel_adcs_batt_out,
};

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */

void ADC_vref_calibration();
void ADC_routine_start();
void read_adc();
void calculate_avarages();
void convert_values();

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

