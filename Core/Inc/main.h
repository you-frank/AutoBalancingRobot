/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RESET_Pin GPIO_PIN_1
#define RESET_GPIO_Port GPIOA
#define DIO0_Pin GPIO_PIN_2
#define DIO0_GPIO_Port GPIOA
#define NSS_Pin GPIO_PIN_3
#define NSS_GPIO_Port GPIOA
#define BRK_L_PB12_Pin GPIO_PIN_12
#define BRK_L_PB12_GPIO_Port GPIOB
#define DIR_L_PB13_Pin GPIO_PIN_13
#define DIR_L_PB13_GPIO_Port GPIOB
#define ENC_L_A_PB14_ITR_Pin GPIO_PIN_14
#define ENC_L_A_PB14_ITR_GPIO_Port GPIOB
#define ENC_L_A_PB14_ITR_EXTI_IRQn EXTI15_10_IRQn
#define ENC_L_B_PB15_Pin GPIO_PIN_15
#define ENC_L_B_PB15_GPIO_Port GPIOB
#define BRK_R_PA8_Pin GPIO_PIN_8
#define BRK_R_PA8_GPIO_Port GPIOA
#define DIR_R_PA9_Pin GPIO_PIN_9
#define DIR_R_PA9_GPIO_Port GPIOA
#define ENC_R_A_PA10_ITR_Pin GPIO_PIN_10
#define ENC_R_A_PA10_ITR_GPIO_Port GPIOA
#define ENC_R_A_PA10_ITR_EXTI_IRQn EXTI15_10_IRQn
#define ENC_R_B_PA11_Pin GPIO_PIN_11
#define ENC_R_B_PA11_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
