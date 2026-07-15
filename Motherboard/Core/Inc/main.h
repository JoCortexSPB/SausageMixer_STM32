/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HMSENSOR_Pin GPIO_PIN_0
#define HMSENSOR_GPIO_Port GPIOA
#define CNTSENSOR_Pin GPIO_PIN_1
#define CNTSENSOR_GPIO_Port GPIOA
#define NEARSENSOR_Pin GPIO_PIN_2
#define NEARSENSOR_GPIO_Port GPIOA
#define FARSENSOR_Pin GPIO_PIN_3
#define FARSENSOR_GPIO_Port GPIOA
#define AD5328_SYNC_Pin GPIO_PIN_4
#define AD5328_SYNC_GPIO_Port GPIOA
#define AD5328_LDAC_Pin GPIO_PIN_6
#define AD5328_LDAC_GPIO_Port GPIOA
#define VF_FRW_CTRL_Pin GPIO_PIN_0
#define VF_FRW_CTRL_GPIO_Port GPIOB
#define VF_REV_CTRL_Pin GPIO_PIN_1
#define VF_REV_CTRL_GPIO_Port GPIOB
#define SRV_ON_CTRL_Pin GPIO_PIN_2
#define SRV_ON_CTRL_GPIO_Port GPIOB
#define SRV_PULS_CTRL_Pin GPIO_PIN_10
#define SRV_PULS_CTRL_GPIO_Port GPIOB
#define SRV_SIGN_CTRL_Pin GPIO_PIN_11
#define SRV_SIGN_CTRL_GPIO_Port GPIOB
#define CNT_ECHO_Pin GPIO_PIN_12
#define CNT_ECHO_GPIO_Port GPIOB
#define SMALLVALVE_CTRL_Pin GPIO_PIN_8
#define SMALLVALVE_CTRL_GPIO_Port GPIOA
#define BIGVALVE_CTRL_Pin GPIO_PIN_9
#define BIGVALVE_CTRL_GPIO_Port GPIOA
#define USB_HPD_Pin GPIO_PIN_10
#define USB_HPD_GPIO_Port GPIOA

#define VF_FRW_START		HAL_GPIO_WritePin(VF_FRW_CTRL_GPIO_Port, VF_FRW_CTRL_Pin, GPIO_PIN_SET)
#define VF_FRW_STOP			HAL_GPIO_WritePin(VF_FRW_CTRL_GPIO_Port, VF_FRW_CTRL_Pin, GPIO_PIN_RESET)
#define VF_REV_START		HAL_GPIO_WritePin(VF_REV_CTRL_GPIO_Port, VF_REV_CTRL_Pin, GPIO_PIN_SET)
#define VF_REV_STOP			HAL_GPIO_WritePin(VF_REV_CTRL_GPIO_Port, VF_REV_CTRL_Pin, GPIO_PIN_RESET)

#define CMD_NONE     0x00

#define CMD_START    0x10
#define CMD_STOP     0x11
#define CMD_AIR      0x12
#define CMD_RESET    0x13

#define CMD_LEFT     0x20
#define CMD_RIGHT    0x21
#define CMD_UP       0x22
#define CMD_DOWN     0x23

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
