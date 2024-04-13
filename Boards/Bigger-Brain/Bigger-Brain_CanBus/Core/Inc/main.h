/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>

#include "NVF_CanBus_Conf.h"

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
#define V2_SENSE_Pin GPIO_PIN_0
#define V2_SENSE_GPIO_Port GPIOA
#define V1_SENSE_Pin GPIO_PIN_1
#define V1_SENSE_GPIO_Port GPIOA
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define USR_LED_Pin GPIO_PIN_8
#define USR_LED_GPIO_Port GPIOA
#define SD1_DET_Pin GPIO_PIN_15
#define SD1_DET_GPIO_Port GPIOA
#define USR_KEY2_Pin GPIO_PIN_0
#define USR_KEY2_GPIO_Port GPIOD
#define USR_KEY1_Pin GPIO_PIN_1
#define USR_KEY1_GPIO_Port GPIOD
#define SPI1_IRQ_Pin GPIO_PIN_6
#define SPI1_IRQ_GPIO_Port GPIOD
#define SPI1_CS_Pin GPIO_PIN_10
#define SPI1_CS_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
