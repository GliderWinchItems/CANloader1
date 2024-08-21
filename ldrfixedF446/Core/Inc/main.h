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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ADDR_SCB_VTOR 0xE000ED08
#define ADDR_FLASH_SIZE 0x1FFF7A22
#define ADDR_UNIQUE_ID  0x1FFF7A10
#define SYSCLOCKFREQ 180000000  

#define BEGIN_FLASH 0x0800C000 // Application's flash start address
#define APP_JMPADDR 0x0800C004 // Location of application's jump address

#define APPERR_APP_ENTRY_OOR (1 << 0) // App out-of-range address: entry
#define APPERR_APP_CRC_ADDR  (1 << 1) // App out-of-range address: crc
#define APPERR_APP_CHK_ADDR  (1 << 2) // App out-of-range address: checksum
#define APPERR_APP_CRC_NE    (1 << 2) // Computed not equal specified: CRC-32
#define APPERR_APP_CHK_NE    (1 << 2) // Computed not equal specified: checksum
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
#define wake_fetgate_Pin GPIO_PIN_13
#define wake_fetgate_GPIO_Port GPIOC
#define LED_GRN_Pin GPIO_PIN_12
#define LED_GRN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
