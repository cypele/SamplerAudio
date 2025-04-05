/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"

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
#define LED_R_Pin GPIO_PIN_13
#define LED_R_GPIO_Port GPIOJ
#define uSD_Detect_Pin GPIO_PIN_15
#define uSD_Detect_GPIO_Port GPIOI
#define LED_G_Pin GPIO_PIN_5
#define LED_G_GPIO_Port GPIOJ
#define DFSDM_DATIN5_Pin GPIO_PIN_11
#define DFSDM_DATIN5_GPIO_Port GPIOC
#define DFSDM_CKOUT_Pin GPIO_PIN_3
#define DFSDM_CKOUT_GPIO_Port GPIOD
#define DFSDM_DATIN1_Pin GPIO_PIN_3
#define DFSDM_DATIN1_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

typedef struct {
    int16_t* data;
    size_t length;
} AudioChunk_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
