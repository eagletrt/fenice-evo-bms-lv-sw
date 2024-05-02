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
        HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
    }
}

void set_time_set(uint8_t status) {
    if (status == 0) {
        HAL_GPIO_WritePin(TIME_SET_GPIO_Port, TIME_SET_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(TIME_SET_GPIO_Port, TIME_SET_Pin, GPIO_PIN_SET);
    }
}

int discharge_state = 0;

/* USER CODE END 1 */

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
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
    HAL_GPIO_WritePin(GPIOB, NC_MCU0_Pin | LTC_CS_Pin | PB4_NC_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, MUX_A2_Pin | MUX_A3_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : PCPin PCPin PCPin */
    GPIO_InitStruct.Pin   = RELAY_Pin | MUX_A0_Pin | MUX_A1_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PBPin PBPin PBPin */
    GPIO_InitStruct.Pin   = NC_MCU0_Pin | LTC_CS_Pin | PB4_NC_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin  = TIME_SET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(TIME_SET_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PAPin PAPin */
    GPIO_InitStruct.Pin   = MUX_A2_Pin | MUX_A3_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin  = MCP_INT_Pin;
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
uint8_t mcp23017_device_address = 0x40;
uint8_t mcp23017_i2c_timeout    = 10;  // ms
HAL_StatusTypeDef hal_status    = HAL_ERROR;
uint8_t iodira_register_value   = 0x00;
uint8_t iodirb_register_value   = 0x00;
uint8_t gpiob_register_value    = 0x00;
uint8_t gpioa_register_value    = 0x00;
uint8_t gpintena_register_value = 0x00;
uint8_t intcona_register_value  = 0x00;
uint8_t defvala_register_value  = 0x00;
uint8_t intcapa_register_value  = 0x00;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == MCP_INT_Pin) {
        mcp_int_fired = 1;
    }
}

int gpio_extender_init(void) {
    hal_status = HAL_ERROR;

    // set port_b pins as output
    iodirb_register_value = 0x00;
    // set port_a pins as input
    iodira_register_value = 0xFF;

    hal_status = HAL_I2C_Mem_Write(
        &hi2c3,
        mcp23017_device_address,
        MCP23017_REGISTER_IODIRB,
        MCP23017_I2C_SIZE,
        &iodirb_register_value,
        MCP23017_I2C_SIZE,
        mcp23017_i2c_timeout);
    if (hal_status != HAL_OK) {
        return MCP23017_ERROR;
    }

    hal_status = HAL_I2C_Mem_Write(
        &hi2c3,
        mcp23017_device_address,
        MCP23017_REGISTER_IODIRA,
        MCP23017_I2C_SIZE,
        &iodira_register_value,
        MCP23017_I2C_SIZE,
        mcp23017_i2c_timeout);
    if (hal_status != HAL_OK) {
        return MCP23017_ERROR;
    }

#if MCP23017_INTERRUPTS_ENABLED
    uint8_t compare_value = 0b00111111;

    mcp23017_set_it_on_all_pins(
        &gpintena_register_value,
        &intcona_register_value,
        &defvala_register_value,
        MCP23017_INT_ENABLED,
        MCP23017_INT_MODE_ON_CHANGE,
        compare_value);

    hal_status = HAL_I2C_Mem_Write(
        &hi2c3,
        mcp23017_device_address,
        MCP23017_REGISTER_GPINTENA,
        MCP23017_I2C_SIZE,
        &gpintena_register_value,
        MCP23017_I2C_SIZE,
        mcp23017_i2c_timeout);
    if (hal_status != HAL_OK) {
        return MCP23017_ERROR;
    }
    hal_status = HAL_I2C_Mem_Write(
        &hi2c3,
        mcp23017_device_address,
        MCP23017_REGISTER_INTCONA,
        MCP23017_I2C_SIZE,
        &intcona_register_value,
        MCP23017_I2C_SIZE,
        mcp23017_i2c_timeout);
    if (hal_status != HAL_OK) {
        return MCP23017_ERROR;
    }
    hal_status = HAL_I2C_Mem_Write(
        &hi2c3,
        mcp23017_device_address,
        MCP23017_REGISTER_DEFVALA,
        MCP23017_I2C_SIZE,
        &defvala_register_value,
        MCP23017_I2C_SIZE,
        mcp23017_i2c_timeout);
    if (hal_status != HAL_OK) {
        return MCP23017_ERROR;
    }
#endif

    return MCP23017_OK;
}

int set_rfe_frg(int state) {
    hal_status = HAL_ERROR;
    mcp23017_set_register_bit(&gpiob_register_value, mcp_controls_bank_b_frg, state);
    mcp23017_set_register_bit(&gpiob_register_value, mcp_controls_bank_b_rfe, state);

    hal_status = HAL_I2C_Mem_Write(
        &hi2c3,
        mcp23017_device_address,
        MCP23017_REGISTER_GPIOB,
        MCP23017_I2C_SIZE,
        &gpiob_register_value,
        MCP23017_I2C_SIZE,
        mcp23017_i2c_timeout);
    if (hal_status != HAL_OK) {
        return MCP23017_ERROR;
    }

    return MCP23017_OK;
}

int set_led(int led1, int led2, int led3) {
    hal_status = HAL_ERROR;
    mcp23017_set_register_bit(&gpiob_register_value, mcp_controls_bank_b_led_0, led1);
    mcp23017_set_register_bit(&gpiob_register_value, mcp_controls_bank_b_led_1, led2);
    mcp23017_set_register_bit(&gpiob_register_value, mcp_controls_bank_b_led_2, led3);

    hal_status = HAL_I2C_Mem_Write(
        &hi2c3,
        mcp23017_device_address,
        MCP23017_REGISTER_GPIOB,
        MCP23017_I2C_SIZE,
        &gpiob_register_value,
        MCP23017_I2C_SIZE,
        mcp23017_i2c_timeout);
    if (hal_status != HAL_OK) {
        return MCP23017_ERROR;
    }

    return MCP23017_OK;
}

int set_discharge(int state) {
    hal_status      = HAL_ERROR;
    discharge_state = state;
    mcp23017_set_register_bit(&gpiob_register_value, mcp_controls_bank_b_discharge, state);

    hal_status = HAL_I2C_Mem_Write(
        &hi2c3,
        mcp23017_device_address,
        MCP23017_REGISTER_GPIOB,
        MCP23017_I2C_SIZE,
        &gpiob_register_value,
        MCP23017_I2C_SIZE,
        mcp23017_i2c_timeout);
    if (hal_status != HAL_OK) {
        return MCP23017_ERROR;
    }

    return MCP23017_OK;
}

void gpio_extender_routine(void) {
#if MCP23017_INTERRUPTS_ENABLED
    if (mcp_int_fired) {
        mcp_int_fired = 0;

        hal_status = HAL_I2C_Mem_Read(
            &hi2c3,
            mcp23017_device_address,
            MCP23017_REGISTER_INTCAPA,
            MCP23017_I2C_SIZE,
            &intcapa_register_value,
            MCP23017_I2C_SIZE,
            mcp23017_i2c_timeout);

        if (hal_status != HAL_OK) {
            // TO-DO: Error
        }

        for (uint8_t i = 0; i <= MCP23017_PINS_N; i++) {
            mcp23017_feedbacks_state[i] = mcp23017_get_register_bit(intcapa_register_value, i);
        }
    }
#else
    hal_status = HAL_I2C_Mem_Read(
        &hi2c3,
        mcp23017_device_address,
        MCP23017_REGISTER_GPIOA,
        MCP23017_I2C_SIZE,
        &gpioa_register_value,
        MCP23017_I2C_SIZE,
        mcp23017_i2c_timeout);

    if (hal_status != HAL_OK) {
        // TO-DO: Error
    }

    for (uint8_t i = 0; i <= MCP23017_PINS_N; i++) {
        mcp23017_feedbacks_state[i] = mcp23017_get_register_bit(gpioa_register_value, i);
    }
#endif
}

/* USER CODE END 2 */
