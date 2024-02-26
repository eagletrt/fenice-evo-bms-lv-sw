/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

void set_relay(uint8_t status) {
  if (status == 0) {
    HAL_GPIO_WritePin(RELAY_GPIO_Port, GPIO_PIN_15, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(RELAY_GPIO_Port, GPIO_PIN_15, GPIO_PIN_SET);
  }
}

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PA6   ------> S_TIM3_CH1
     PA7   ------> S_TIM3_CH2
     PB0   ------> S_TIM3_CH3
     PB1   ------> S_TIM3_CH4
     PC6   ------> S_TIM8_CH1
*/
void MX_GPIO_Init(void) {

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RELAY_Pin | MUX_A0_Pin | MUX_A1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NC_MCU0_Pin | LTC_CS_Pin | PB4_NC_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MUX_A2_Pin | MUX_A3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = RELAY_Pin | MUX_A0_Pin | MUX_A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = RAD_L_Pin | RAD_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = FAN_Pin | STP_DIRECTION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = NC_MCU0_Pin | LTC_CS_Pin | PB4_NC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = TIME_SET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TIME_SET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = MUX_A2_Pin | MUX_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = MCP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MCP_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

/* USER CODE BEGIN 2 */

int mcp_int_fired = 0;
uint8_t mcp23017_feedbacks_state[8];
uint8_t mcp23017_device_address = 0x00;
uint8_t mcp23017_i2c_timeout = 10; // ms
HAL_StatusTypeDef HAL_Status = HAL_ERROR;
uint8_t gpinten_register_value = 0b00000000;
uint8_t gpiob_register_value = 0b00000000;
uint8_t gpioa_register_value = 0b00000000;
uint8_t intcapa_register_value = 0b00000000;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == MCP_INT_Pin) {
    mcp_int_fired = 1;
  }
}

int gpio_extender_init(void) {
  HAL_Status = HAL_ERROR;

  /**
   * This can also be done with
   * mcp23017_set_register_bit()
   * or
   * mcp23017_set_it_on_pin()
   */
  gpinten_register_value = 0b11111111;

  HAL_Status = HAL_I2C_Mem_Write(&hi2c3, mcp23017_device_address,
                                 MCP23017_REGISTER_GPINTENA, MCP23017_I2C_SIZE,
                                 &gpinten_register_value, MCP23017_I2C_SIZE,
                                 mcp23017_i2c_timeout);
  if (HAL_Status != HAL_OK) {
    return MCP23017_ERROR;
  }

  return MCP23017_OK;
}

int set_rfe_frg(int state) {
  HAL_Status = HAL_ERROR;
  mcp23017_set_register_bit(&gpiob_register_value, mcp_controls_bank_b_frg,
                            state);
  mcp23017_set_register_bit(&gpiob_register_value, mcp_controls_bank_b_rfe,
                            state);

  HAL_Status = HAL_I2C_Mem_Write(&hi2c3, mcp23017_device_address,
                                 MCP23017_REGISTER_GPIOB, MCP23017_I2C_SIZE,
                                 &gpiob_register_value, MCP23017_I2C_SIZE,
                                 mcp23017_i2c_timeout);
  if (HAL_Status != HAL_OK) {
    return MCP23017_ERROR;
  }

  return MCP23017_OK;
}

int set_led(int led1, int led2, int led3) {
  HAL_Status = HAL_ERROR;
  mcp23017_set_register_bit(&gpiob_register_value, mcp_controls_bank_b_led_0,
                            led1);
  mcp23017_set_register_bit(&gpiob_register_value, mcp_controls_bank_b_led_1,
                            led2);
  mcp23017_set_register_bit(&gpiob_register_value, mcp_controls_bank_b_led_2,
                            led3);

  HAL_Status = HAL_I2C_Mem_Write(&hi2c3, mcp23017_device_address,
                                 MCP23017_REGISTER_GPIOB, MCP23017_I2C_SIZE,
                                 &gpiob_register_value, MCP23017_I2C_SIZE,
                                 mcp23017_i2c_timeout);
  if (HAL_Status != HAL_OK) {
    return MCP23017_ERROR;
  }

  return MCP23017_OK;
}

int set_discharge(int state) {
  HAL_Status = HAL_ERROR;
  mcp23017_set_register_bit(&gpiob_register_value,
                            mcp_controls_bank_b_discharge, state);

  HAL_Status = HAL_I2C_Mem_Write(&hi2c3, mcp23017_device_address,
                                 MCP23017_REGISTER_GPIOB, MCP23017_I2C_SIZE,
                                 &gpiob_register_value, MCP23017_I2C_SIZE,
                                 mcp23017_i2c_timeout);
  if (HAL_Status != HAL_OK) {
    return MCP23017_ERROR;
  }

  return MCP23017_OK;
}

void gpio_extender_routine(void) {
  if (mcp_int_fired) {
    mcp_int_fired = 0;
    HAL_Status = HAL_I2C_Mem_Read(&hi2c3, mcp23017_device_address,
                                  MCP23017_REGISTER_INTCAPA, MCP23017_I2C_SIZE,
                                  &intcapa_register_value, MCP23017_I2C_SIZE,
                                  mcp23017_i2c_timeout);
    if (HAL_Status != HAL_OK) {
      // TO-DO: Error
    }
    for (uint8_t i = 0; i < 8; i++) {
      mcp23017_feedbacks_state[i] =
          mcp23017_get_register_bit(intcapa_register_value, i);
    }
  }
}

/* USER CODE END 2 */
