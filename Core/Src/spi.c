/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    spi.c
 * @brief   This file provides code for the configuration
 *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;

/* SPI2 init function */
void MX_SPI2_Init(void) {

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle) {

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (spiHandle->Instance == SPI2) {
    /* USER CODE BEGIN SPI2_MspInit 0 */

    /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = LTC_CLK_Pin | LCT_MISO_Pin | LTC_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI2_MspInit 1 */

    /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle) {

  if (spiHandle->Instance == SPI2) {
    /* USER CODE BEGIN SPI2_MspDeInit 0 */

    /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, LTC_CLK_Pin | LCT_MISO_Pin | LTC_MOSI_Pin);

    /* USER CODE BEGIN SPI2_MspDeInit 1 */

    /* USER CODE END SPI2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

#include "ltc6811.h"

#define LTC_COUNT 1
#define MONITORED_VOLTAGES                                                     \
  (((LTC6811_CVDR + 1) * LTC6811_REG_CELL_COUNT) * LTC_COUNT)
float cell_voltages[MONITORED_VOLTAGES];
Ltc6811Chain chain;
Ltc6811Cfgr config[LTC_COUNT];

void monitor_init(void) {
  for (size_t i = 0; i < LTC_COUNT; ++i) {
    config[i].ADCOPT = 1;
    config[i].DTEN = 1;
    config[i].REFON = 1;
    config[i].GPIO = 0b00000;
    config[i].VUV = 0;
    config[i].VOV = 0;
    config[i].DCC = 0;
    config[i].DCTO = 0;
  }
  ltc6811_chain_init(&chain, LTC_COUNT);

  HAL_GPIO_WritePin(LTC_CS_GPIO_Port, LTC_CS_Pin, GPIO_PIN_SET);
}

void monitor_routine(void) {
  uint8_t conv_data[LTC6811_POLL_BUFFER_SIZE(LTC_COUNT)];
  size_t byte_count = ltc6811_adcv_encode_broadcast(
      &chain, LTC6811_MD_27KHZ_14KHZ, LTC6811_DCP_DISABLED, LTC6811_CH_ALL,
      conv_data);
  HAL_GPIO_WritePin(LTC_CS_GPIO_Port, LTC_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, conv_data, byte_count, 10);
  HAL_GPIO_WritePin(LTC_CS_GPIO_Port, LTC_CS_Pin, GPIO_PIN_SET);

  /**
   * TODO: replace the HAL_Delay() and use the correct polling function
   */
  HAL_Delay(10);

  for (size_t ltc_rgs = LTC6811_CVAR; ltc_rgs <= LTC6811_CVDR; ++ltc_rgs) {
    uint8_t rd_data[LTC6811_READ_BUFFER_SIZE(LTC_COUNT)];
    uint8_t volt_data[8];
    byte_count = ltc6811_rdcv_encode_broadcast(&chain, ltc_rgs, rd_data);
    HAL_GPIO_WritePin(LTC_CS_GPIO_Port, LTC_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, rd_data, byte_count, 10);
    HAL_SPI_Receive(&hspi2, volt_data, 8, 10);
    HAL_GPIO_WritePin(LTC_CS_GPIO_Port, LTC_CS_Pin, GPIO_PIN_SET);

    uint16_t volts[LTC6811_REG_CELL_COUNT] = {0};
    ltc6811_rdcv_decode_broadcast(&chain, volt_data, volts);
    for (size_t idx = 0; idx < LTC6811_REG_CELL_COUNT; idx++) {
      cell_voltages[LTC6811_CVDR * ltc_rgs + idx] = (float)volts[idx] / 10000.f;
    }
  }
}

/* USER CODE END 1 */
