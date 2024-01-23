/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "dac.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bms_fsm.h"
#include "can_manager.h"
#include "can_messages.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_FilterTypeDef primary_filter = {
  .FilterMode       = CAN_FILTERMODE_IDMASK,
  .FilterIdLow      = 0 << 5,                 // Take all ids from 0
  .FilterIdHigh     = ((1U << 11) - 1) << 5,  // to 2^11 - 1
  .FilterMaskIdHigh = 0 << 5,                 // Don't care on can id bits
  .FilterMaskIdLow  = 0 << 5,                 // Don't care on can id bits
  /* HAL considers IdLow and IdHigh not as just the ID of the can message but
      as the combination of: 
      STDID + RTR + IDE + 4 most significant bits of EXTID
  */
  .FilterFIFOAssignment = CAN_FILTER_FIFO0,
  .FilterBank           = 0,
  .FilterScale          = CAN_FILTERSCALE_16BIT,
  .FilterActivation     = ENABLE,
  .SlaveStartFilterBank = 14,
};

CAN_FilterTypeDef seconday_filter = {
  .FilterMode       = CAN_FILTERMODE_IDMASK,
  .FilterIdLow      = 0 << 5,                 // Take all ids from 0
  .FilterIdHigh     = ((1U << 11) - 1) << 5,  // to 2^11 - 1
  .FilterMaskIdHigh = 0 << 5,                 // Don't care on can id bits
  .FilterMaskIdLow  = 0 << 5,                 // Don't care on can id bits
  /* HAL considers IdLow and IdHigh not as just the ID of the can message but
      as the combination of: 
      STDID + RTR + IDE + 4 most significant bits of EXTID
  */
  .FilterFIFOAssignment = CAN_FILTER_FIFO1,
  .FilterBank           = 14,
  .FilterScale          = CAN_FILTERSCALE_16BIT,
  .FilterActivation     = ENABLE,
  .SlaveStartFilterBank = 14,
};

extern bool start_dma_read;
extern bool start_value_conversion;
extern bool start_calculating_averages;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN2_Init();
  MX_DAC_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_TIM10_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //  MX_DMA_Init() must be executed before MX_ADC_Init() otherwise the ADC doesnt' work in DMA mode correctly
  //  but since __CUBEMIX e' stronzo__(cit.), this isn't enforced by the code generator
  //  threfore manual intervention is necessary
  //  https://community.st.com/s/question/0D50X0000ALudz8SQB/how-to-enable-adc-continuous-mode-with-dma
  
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  state_t cur_state = STATE_INIT;

  extern HAL_StatusTypeDef can_manager_hal_status_retval;
  extern int can_manager_error_code;
  int primary_can_id = can_init(&hcan1, can_primary_ntw_handler, CAN_IT_ERROR | CAN_IT_RX_FIFO0_MSG_PENDING, &primary_filter);
  if (can_manager_hal_status_retval != HAL_OK)
  {
    can_init_errors_handler(can_manager_error_code);
  }
  

  int secondary_can_id = can_init(&hcan2, can_secondary_ntw_handler, CAN_IT_ERROR | CAN_IT_RX_FIFO1_MSG_PENDING, &seconday_filter);
  if (can_manager_hal_status_retval != HAL_OK)
  {
    can_init_errors_handler(can_manager_error_code);
  }

  ADC_routine_start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    cur_state = run_state(cur_state, NULL);

    consume_rx_queue(primary_can_id);
    consume_rx_queue(secondary_can_id);
    flush_tx_queue(primary_can_id);
    flush_tx_queue(secondary_can_id);

    if (start_dma_read)
    {
      read_adc();
    }
    if (start_calculating_averages)
    {
      calculate_avarages();
    }
    if (start_value_conversion)
    {
      convert_values();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
