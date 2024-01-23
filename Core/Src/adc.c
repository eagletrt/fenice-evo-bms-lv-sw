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
#include "tim.h"
#include <stdbool.h>
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
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
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
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
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
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
    GPIO_InitStruct.Pin = AS_COMPUTER_FB_Pin|REAY_OUT_ANALOG_FB_Pin|LVMS_OUT_ANALOG_FB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MUX_FB_OUT_Pin|MUX_HALL_Pin|BATT_OUT_ANALOG_FB_Pin;
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
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc2);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
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
  }
  else if(adcHandle->Instance==ADC2)
  {
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
    HAL_GPIO_DeInit(GPIOC, AS_COMPUTER_FB_Pin|REAY_OUT_ANALOG_FB_Pin|LVMS_OUT_ANALOG_FB_Pin);

    HAL_GPIO_DeInit(GPIOA, MUX_FB_OUT_Pin|MUX_HALL_Pin|BATT_OUT_ANALOG_FB_Pin);

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
//use the same array for all steps?
uint16_t adc2_raw_values[ADC2_CHANNELS_N] = {0};
uint8_t adc2_channels_len = sizeof(adc2_raw_values) / sizeof(adc2_raw_values[0]);
uint32_t fb_raw_values[MUX_CHANNELS_N] = {0};
uint32_t hall_raw_values[MUX_CHANNELS_N] = {0};
uint32_t fb_average_values[MUX_CHANNELS_N] = {0};
uint32_t hall_average_values[MUX_CHANNELS_N] = {0};
float fb_converted_values[MUX_CHANNELS_N] = {0};
float hall_converted_values[MUX_CHANNELS_N] = {0};
//using the same variable for all steps
uint32_t computer_fb = 0;
uint32_t relay_out = 0;
uint32_t lvms_out = 0;
uint32_t batt_out = 0;
//---
float vref;

// routine process flags
bool is_adc_dma_complete = false;
bool start_dma_read = false;
bool start_value_conversion = false;
bool start_calculating_averages = false;
bool vref_samples_acquired = false;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  if (hadc == &hadc1) {
    is_adc_dma_complete = true;
  } else if (hadc == &hadc2) {
    vref_samples_acquired = true;
  }
}

void ADC_routine(TIM_HandleTypeDef *htim){
  start_dma_read = true;
}

void ADC_routine_start() {
  //Start timer for adc2 readings (vref)
  HAL_TIM_Base_Start_IT(&htim10); //TIMER_ADC_MEAS

  HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, &ADC_routine);
  //Start timer for ADC_routine
  HAL_TIM_Base_Start_IT(&htim6);
}

void set_address(uint8_t multiplexer_channel_address){
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, (multiplexer_channel_address >> 0) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, (multiplexer_channel_address >> 1) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (multiplexer_channel_address >> 2) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (multiplexer_channel_address >> 3) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t ADC_get_resolution_bits(ADC_HandleTypeDef* adcHandle) {
    
    // comments found in the old code:
    // To get this value look at the reference manual RM0390
    // page 385 section 13.13.2 register ADC_CR1 RES[1:0] bits
    
    uint8_t ret = 12;
    switch (ADC_GET_RESOLUTION(adcHandle)) {
        case 0U:
            ret = 12U;
            break;
        case 1U:
            ret = 10U;
            break;
        case 2U:
            ret = 8U;
            break;
        case 4U:
            ret = 6U;
            break;
    }
    return ret;
}

uint32_t ADC_get_tot_voltage_levels(ADC_HandleTypeDef* adcHandle) {
    uint8_t value = ADC_get_resolution_bits(adcHandle);
    return ((uint32_t)(1U << value) - 1);  // 2^value - 1
}

float ADC_get_value_mV(ADC_HandleTypeDef *adcHandle, uint32_t value_from_adc) {
  
  //define in the old code
  const uint16_t ADC_FSVR_mV = 3300U;

  return value_from_adc * ((float)ADC_FSVR_mV / ADC_get_tot_voltage_levels(adcHandle));
}

// See Documentation/STM32_VREF.pdf for more information
void ADC_vref_calibration() {
    uint16_t buffer[2 * 500] = {0}; //N_ADC_CALIBRATION_CHANNELS * N_ADC_CALIBRATION_SAMPLES
    uint32_t vdda = 0;
    uint32_t vref_int = 0;
    uint16_t *factory_calibration = (uint16_t *)0x1FFF7A2A;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //TIMER_ADC_CALIBRATION, TIMER_ADC_CALIBRATION_CHANNEL
    // CALIBRATION_ADC, -, N_ADC_CALIBRATION_SAMPLES, N_ADC_CALIBRATION_SAMPLES
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&buffer, 2 * 500) != HAL_OK) {
        // error_set(ERROR_ADC_INIT, 0);
        // TO-DO generate error
    } else {
        // error_reset(ERROR_ADC_INIT, 0);
        // TO-DO generate error
    }

    //Wait until 500 samples has been reached
    while (!vref_samples_acquired) {}

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    for (uint16_t i = 0; i < 500; i++) { //N_ADC_CALIBRATION_SAMPLES
      vref_int += buffer[2 * i]; //N_ADC_CALIBRATION_CHANNELS
      vdda += buffer[2 * i + 1]; //N_ADC_CALIBRATION_CHANNELS
    }

    vdda /= 500;
    vref_int /= 500;

    vref = ADC_get_value_mV(&hadc1, vdda) * ((float)*factory_calibration / vref_int);
    
    // if (vref < 3100 || vref > 3300) {
    //     error_set(ERROR_ADC_INIT, 0);
    // }
    // TO-DO generate error if vref is not plausible
}

float ADC_get_calibrated_mV(ADC_HandleTypeDef *adcHandle, uint32_t value_from_adc) {
    return value_from_adc * ((float)vref / ADC_get_tot_voltage_levels(adcHandle));
}

float calculate_current_mA(uint32_t value_from_adc) {
    float adc_val_mV = ADC_get_calibrated_mV(&hadc2, value_from_adc);

    // current [mA] = ((Vadc-Vref)[mV] / Sensibility [mV/A])*1000
    
    // defines in the old code 
    const uint16_t HO_50_SP33_1106_VREF_mV = 1645U;
    const float HO_50_SP33_1106_THEORETICAL_SENSITIVITY = 9.2f;

    float current = ((adc_val_mV - HO_50_SP33_1106_VREF_mV) / HO_50_SP33_1106_THEORETICAL_SENSITIVITY) * 1000;
    return current;
}

float current_transducer_get_electric_current_mA(uint32_t value_from_adc) {
    float current_in_mA = calculate_current_mA(value_from_adc);
    
    //#define CT_OVERCURRENT_THRESHOLD_MA (50000U)
    //is_overcurrent = (current_in_ma > CT_OVERCURRENT_THRESHOLD_MA);
    
    //TO-DO generate error if overcurrent 

    return current_in_mA;
}

// Stack OverMazzucchi
// https://github.com/eagletrt/not-yet-acquisinator-sw/blob/master/Core/Src/acquisinator.c

uint32_t mean_values[MEAN_VALUES_ARRAY_LEN] = {0};
uint32_t adc2_history[MEAN_VALUES_ARRAY_LEN][HISTORY_L] = {0};
uint8_t adc2_filled[MEAN_VALUES_ARRAY_LEN] = {0};
uint32_t adc2_curri[MEAN_VALUES_ARRAY_LEN] = {0};

int moving_avg(int cidx) {
  if (adc2_filled[cidx]) {
    mean_values[cidx] = mean_values[cidx] -
                        (adc2_history[cidx][adc2_curri[cidx]]) +
                        (adc2_raw_values[cidx] / HISTORY_L);
    adc2_history[cidx][adc2_curri[cidx]] = adc2_raw_values[cidx] / HISTORY_L;
  } else {
    mean_values[cidx] += (adc2_raw_values[cidx] / HISTORY_L);
    adc2_history[cidx][adc2_curri[cidx]] = adc2_raw_values[cidx] / HISTORY_L;
  }
  ++adc2_curri[cidx];
  if (adc2_curri[cidx] == HISTORY_L) {
    adc2_curri[cidx] = 0;
    adc2_filled[cidx] = 1;
  }
  if (adc2_filled[cidx]) {
    return mean_values[cidx];
  }
  return -1;
}

void read_adc(){
  start_dma_read = false;

  HAL_ADC_Start_DMA(&hadc1, &adc2_raw_values, adc2_channels_len);
  while (!is_adc_dma_complete) {
    continue;
  }
  
  hall_raw_values[0] = adc2_raw_values[adc2_channel_mux_hall];
  fb_raw_values[0] = adc2_raw_values[adc2_channel_mux_fb];
  
  //this values need to be read only once
  computer_fb = adc2_raw_values[adc2_channel_adcs_as_computer_fb];
  relay_out = adc2_raw_values[adc2_channel_adcs_relay_out];
  lvms_out = adc2_raw_values[adc2_channel_adcs_lvms_out];
  batt_out = adc2_raw_values[adc2_channel_adcs_batt_out];

  for (uint8_t i = 1; i < MUX_CHANNELS_N; i++) {  
    set_address(i);

    while (!is_adc_dma_complete) {
      continue;
    }
    is_adc_dma_complete = false;

    hall_raw_values[i] = adc2_raw_values[adc2_channel_mux_hall];
    fb_raw_values[i] = adc2_raw_values[adc2_channel_mux_fb];

    HAL_ADC_Start_DMA(&hadc1, &adc2_raw_values, adc2_channels_len);
  }

  start_calculating_averages = true;
}

void calculate_avarages(){
  start_calculating_averages = false;

  for (uint8_t i = 0; i < MUX_CHANNELS_N; i++)
  {
    fb_average_values[i] = moving_avg(i);
    hall_average_values[i] = moving_avg(MUX_CHANNELS_N + i); 
  }
  
  computer_fb = moving_avg(MUX_CHANNELS_N * 2);
  relay_out = moving_avg(MUX_CHANNELS_N * 2 + 1);
  lvms_out = moving_avg(MUX_CHANNELS_N * 2 + 2);
  batt_out = moving_avg(MUX_CHANNELS_N * 2 + 3);
  
  start_value_conversion = true;
}

void convert_values(){
  start_value_conversion = false;

  for (uint8_t i = 0; i < MUX_CHANNELS_N; i++)
  {
    fb_converted_values[i] = ADC_get_calibrated_mV(&hadc1, fb_average_values[i]);

    //S_HALL0 is currently not used
    //if (i == S_HALL0) {}
    
    if (i == S_HALL1) {
      hall_converted_values[i] = current_transducer_get_electric_current_mA(hall_average_values[MUX_CHANNELS_N + 1]) - S_HALL1_OFFSET_mA;
    } else if (i == S_HALL2) {
      hall_converted_values[i] = current_transducer_get_electric_current_mA(hall_average_values[MUX_CHANNELS_N + 1]) - S_HALL2_OFFSET_mA;
    } else {
      hall_converted_values[i] = current_transducer_get_electric_current_mA(hall_average_values[MUX_CHANNELS_N + 1]);
    }
  }

  computer_fb = ADC_get_calibrated_mV(&hadc1, computer_fb) * ADC2_VOLTAGE_DIVIDER_MULTIPLIER;
  relay_out = ADC_get_calibrated_mV(&hadc1, relay_out) * ADC2_VOLTAGE_DIVIDER_MULTIPLIER;
  lvms_out = ADC_get_calibrated_mV(&hadc1, lvms_out) * ADC2_VOLTAGE_DIVIDER_MULTIPLIER;
  batt_out = ADC_get_calibrated_mV(&hadc1, batt_out) * ADC2_VOLTAGE_DIVIDER_MULTIPLIER;
}

void send_to_can(){}
/* USER CODE END 1 */
