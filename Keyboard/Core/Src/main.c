#include "main.h"
#include "wh0802d.h"
#include <string.h>

SPI_HandleTypeDef hspi1;
WH0802D_HandleTypeDef lcd;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

void Keyboard_SendCommand(uint8_t cmd);
void Keyboard_ProcessButtons(void);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();

  lcd.data_port = GPIOB;
  lcd.ctrl_port = GPIOB;

  lcd.rs_pin = GPIO_PIN_8;
  lcd.rw_pin = GPIO_PIN_9;
  lcd.e_pin = GPIO_PIN_10;

  WH0802D_Init(&lcd);

  WH0802D_Clear(&lcd);
  WH0802D_SetCursor(&lcd, 0, 0);
  WH0802D_WriteString(&lcd, "HELLO");

  while (1)
  {
	  Keyboard_ProcessButtons();
  }
}

void Keyboard_SendCommand(uint8_t cmd)
{
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
}

void Keyboard_ProcessButtons(void)
{
	WH0802D_Clear(&lcd);
	WH0802D_SetCursor(&lcd, 0, 0);

    if (HAL_GPIO_ReadPin(BTN_START_GPIO_Port, BTN_START_Pin) == GPIO_PIN_SET)
    {
        Keyboard_SendCommand(CMD_START);
        WH0802D_WriteString(&lcd, "START");
        HAL_Delay(200);
    } else if (HAL_GPIO_ReadPin(BTN_STOP_GPIO_Port, BTN_STOP_Pin) == GPIO_PIN_RESET)
    {
        Keyboard_SendCommand(CMD_STOP);
        WH0802D_WriteString(&lcd, "STOP");
        HAL_Delay(200);
    } else if (HAL_GPIO_ReadPin(BTN_AIR_GPIO_Port, BTN_AIR_Pin) == GPIO_PIN_SET)
    {
        Keyboard_SendCommand(CMD_AIR);
        WH0802D_WriteString(&lcd, "AIR");
        HAL_Delay(200);
    } else if (HAL_GPIO_ReadPin(BTN_RESET_GPIO_Port, BTN_RESET_Pin) == GPIO_PIN_SET)
    {
        Keyboard_SendCommand(CMD_RESET);
        WH0802D_WriteString(&lcd, "RESET");
        HAL_Delay(200);
    } else if (HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin) == GPIO_PIN_SET)
    {
        Keyboard_SendCommand(CMD_LEFT);
        WH0802D_WriteString(&lcd, "LEFT");
        HAL_Delay(200);
    } else if (HAL_GPIO_ReadPin(BTN_RIGHT_GPIO_Port, BTN_RIGHT_Pin) == GPIO_PIN_SET)
    {
        Keyboard_SendCommand(CMD_RIGHT);
        WH0802D_WriteString(&lcd, "RIGHT");
        HAL_Delay(200);
    } else if (HAL_GPIO_ReadPin(BTN_UP_GPIO_Port, BTN_UP_Pin) == GPIO_PIN_SET)
    {
        Keyboard_SendCommand(CMD_UP);
        WH0802D_WriteString(&lcd, "UP");
        HAL_Delay(200);
    } else if (HAL_GPIO_ReadPin(BTN_DOWN_GPIO_Port, BTN_DOWN_Pin) == GPIO_PIN_SET)
    {
        Keyboard_SendCommand(CMD_DOWN);
        WH0802D_WriteString(&lcd, "DOWN");
        HAL_Delay(200);
    } else {
    	WH0802D_WriteString(&lcd, "NONE");
    	HAL_Delay(200);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD0_Pin|LD1_Pin|LD2_Pin|LE_Pin
                          |LD3_Pin|LD4_Pin|LD5_Pin|LD6_Pin
                          |LD7_Pin|LRS_Pin|LRW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LAMP_GPIO_Port, LAMP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD0_Pin LD1_Pin LD2_Pin LE_Pin
                           LD3_Pin LD4_Pin LD5_Pin LD6_Pin
                           LD7_Pin LRS_Pin LRW_Pin */
  GPIO_InitStruct.Pin = LD0_Pin|LD1_Pin|LD2_Pin|LE_Pin
                          |LD3_Pin|LD4_Pin|LD5_Pin|LD6_Pin
                          |LD7_Pin|LRS_Pin|LRW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_START_Pin BTN_STOP_Pin BTN_AIR_Pin BTN_RESET_Pin */
  GPIO_InitStruct.Pin = BTN_START_Pin|BTN_STOP_Pin|BTN_AIR_Pin|BTN_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_LEFT_Pin BTN_RIGHT_Pin BTN_UP_Pin BTN_DOWN_Pin */
  GPIO_InitStruct.Pin = BTN_LEFT_Pin|BTN_RIGHT_Pin|BTN_UP_Pin|BTN_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LAMP_Pin */
  GPIO_InitStruct.Pin = LAMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LAMP_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
