/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    adc.c
 * @brief   This file provides code for the configuration
 *          of the ADC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "current_transducer.h"

#include "usart.h"
#include <stdio.h>

#define FACTORY_CALIBRATION_ADD ((uint16_t *)0x1FFF7A2A)
#define DEFAULT_ADC_VREF (3300U)
#define S_HALL1_OFFSET_mA (300.0f)
#define S_HALL2_OFFSET_mA (1500.0f)
#define ADC2_VDM (9.0f) // Voltage Divider Multiplier

int current_mux_idx = 0, adc2_conversion_ended = 0,
    vref_calibration_conversion_ended = 0;
const uint8_t mux_addresses[16] = {0, 8, 4, 12, 2, 10, 6, 14,
                                   1, 9, 5, 13, 3, 11, 7, 15};
const uint16_t mux_ctrl_pins[directly_connected_fbs_n_values] = {
    MUX_A0_Pin, MUX_A1_Pin, MUX_A2_Pin, MUX_A3_Pin};
GPIO_TypeDef *mux_ctrl_ports[directly_connected_fbs_n_values] = {
    MUX_A0_GPIO_Port, MUX_A1_GPIO_Port, MUX_A2_GPIO_Port, MUX_A3_GPIO_Port};
uint32_t adc2_raw[adc2_ch_n_values] = {0};
float mux_fb_mV[mux_fb_n_values];
float mux_sensors_mA[mux_sensors_n_values];
float dc_fb_mV[directly_connected_fbs_n_values];
float global_bms_lv_vref = DEFAULT_ADC_VREF;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc == &hadc2) {
    adc2_conversion_ended = 1;
  } else if (hadc == &hadc1) {
    vref_calibration_conversion_ended = 1;
  }
}

void set_mux_addr(void) {
  uint8_t cmux_addr = mux_addresses[current_mux_idx];
  for (size_t midx = 0; midx < 4; midx++) {
    HAL_GPIO_WritePin(mux_ctrl_ports[midx], mux_ctrl_pins[midx],
                      ((cmux_addr >> midx) & 1) ? GPIO_PIN_SET
                                                : GPIO_PIN_RESET);
  }
}

void adc_routine_start(void) {
  for (size_t idx = 0; idx < mux_fb_n_values; idx++)
    mux_fb_mV[idx] = FLOAT_UNINITIALIZED_VALUE;
  for (size_t idx = 0; idx < mux_sensors_n_values; idx++)
    mux_sensors_mA[idx] = FLOAT_UNINITIALIZED_VALUE;
  for (size_t idx = 0; idx < directly_connected_fbs_n_values; idx++)
    dc_fb_mV[idx] = FLOAT_UNINITIALIZED_VALUE;

  set_mux_addr();
  if (HAL_ADC_Start_DMA(&hadc2, adc2_raw, adc2_ch_n_values) != HAL_OK) {
    // TODO handle error
  };
  // TODO: start vrefint calibration
}

// TODO: first check that the acquisition works, then add vrefint calibration
void adc_vrefint_calibration(void) {}

float to_mV(uint32_t raw_value) {
  return (float)raw_value * (global_bms_lv_vref / 4096.0f);
}

void adc_routine(void) {
  if (adc2_conversion_ended) {
    if (current_mux_idx < 8) {
      float mux_converted_val = to_mV(adc2_raw[adc2_ch_mux_hall_idx]);
      if (current_mux_idx == mux_sensors_s_hall1_idx) {
        mux_converted_val =
            CT_get_electric_current_mA(mux_converted_val) - S_HALL1_OFFSET_mA;
      } else if (current_mux_idx == mux_sensors_s_hall2_idx) {
        mux_converted_val =
            CT_get_electric_current_mA(mux_converted_val) - S_HALL2_OFFSET_mA;
      }
      mux_sensors_mA[current_mux_idx] = mux_converted_val;
    }
    mux_fb_mV[current_mux_idx] = to_mV(adc2_raw[adc2_ch_mux_fb_idx]) * ADC2_VDM;
    dc_fb_mV[fb_computer_fb_idx] =
        to_mV(adc2_raw[adc2_ch_not_used_idx]) * ADC2_VDM;
    dc_fb_mV[fb_relay_out_idx] =
        to_mV(adc2_raw[adc2_ch_relay_out_idx]) * ADC2_VDM;
    dc_fb_mV[fb_lvms_out_idx] =
        to_mV(adc2_raw[adc2_ch_lvms_out_idx]) * ADC2_VDM;
    dc_fb_mV[fb_batt_out_idx] =
        to_mV(adc2_raw[adc2_ch_batt_out_idx]) * ADC2_VDM;

    current_mux_idx = (current_mux_idx + 1) % 16;
    set_mux_addr();
    if (HAL_ADC_Start_DMA(&hadc2, adc2_raw, adc2_ch_n_values) != HAL_OK) {
      // TODO handle error
    }
    adc2_conversion_ended = 0;
  }
  if (vref_calibration_conversion_ended) {
    vref_calibration_conversion_ended = 0;
  }

  // char buffer[50];
  // int size = sprintf(buffer, "HALL OCD 0: %f\r\n", mux_sensors_mA[mux_sensors_hall_0cd0_idx]);
  // HAL_UART_Transmit(&huart1, buffer, size, 10);
}

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

/* ADC1 init function */
void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}
/* ADC2 init function */
void MX_ADC2_Init(void) {

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 6;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle) {

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (adcHandle->Instance == ADC1) {
    /* USER CODE BEGIN ADC1_MspInit 0 */

    /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA2     ------> ADC1_IN2
    */
    GPIO_InitStruct.Pin = VREF_ADC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VREF_ADC_GPIO_Port, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA2_Stream4;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
    /* USER CODE BEGIN ADC1_MspInit 1 */

    /* USER CODE END ADC1_MspInit 1 */
  } else if (adcHandle->Instance == ADC2) {
    /* USER CODE BEGIN ADC2_MspInit 0 */

    /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PC3     ------> ADC2_IN13
    PA0-WKUP     ------> ADC2_IN0
    PA1     ------> ADC2_IN1
    PA3     ------> ADC2_IN3
    PC4     ------> ADC2_IN14
    PC5     ------> ADC2_IN15
    */
    GPIO_InitStruct.Pin =
        AS_COMPUTER_FB_Pin | REAY_OUT_ANALOG_FB_Pin | LVMS_OUT_ANALOG_FB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        MUX_FB_OUT_Pin | MUX_HALL_Pin | BATT_OUT_ANALOG_FB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA2_Stream2;
    hdma_adc2.Init.Channel = DMA_CHANNEL_1;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK) {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc2);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
    /* USER CODE BEGIN ADC2_MspInit 1 */

    /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle) {

  if (adcHandle->Instance == ADC1) {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA2     ------> ADC1_IN2
    */
    HAL_GPIO_DeInit(VREF_ADC_GPIO_Port, VREF_ADC_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
    /* USER CODE BEGIN ADC1:ADC_IRQn disable */
    /**
     * Uncomment the line below to disable the "ADC_IRQn" interrupt
     * Be aware, disabling shared interrupt may affect other IPs
     */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
    /* USER CODE END ADC1:ADC_IRQn disable */

    /* USER CODE BEGIN ADC1_MspDeInit 1 */

    /* USER CODE END ADC1_MspDeInit 1 */
  } else if (adcHandle->Instance == ADC2) {
    /* USER CODE BEGIN ADC2_MspDeInit 0 */

    /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();

    /**ADC2 GPIO Configuration
    PC3     ------> ADC2_IN13
    PA0-WKUP     ------> ADC2_IN0
    PA1     ------> ADC2_IN1
    PA3     ------> ADC2_IN3
    PC4     ------> ADC2_IN14
    PC5     ------> ADC2_IN15
    */
    HAL_GPIO_DeInit(GPIOC, AS_COMPUTER_FB_Pin | REAY_OUT_ANALOG_FB_Pin |
                               LVMS_OUT_ANALOG_FB_Pin);

    HAL_GPIO_DeInit(GPIOA,
                    MUX_FB_OUT_Pin | MUX_HALL_Pin | BATT_OUT_ANALOG_FB_Pin);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC2 interrupt Deinit */
    /* USER CODE BEGIN ADC2:ADC_IRQn disable */
    /**
     * Uncomment the line below to disable the "ADC_IRQn" interrupt
     * Be aware, disabling shared interrupt may affect other IPs
     */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
    /* USER CODE END ADC2:ADC_IRQn disable */

    /* USER CODE BEGIN ADC2_MspDeInit 1 */

    /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
