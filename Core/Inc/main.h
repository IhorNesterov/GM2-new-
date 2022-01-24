/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define Control_LED_Pin GPIO_PIN_13
#define Control_LED_GPIO_Port GPIOC
#define DS18_Pin GPIO_PIN_2
#define DS18_GPIO_Port GPIOA
#define HV_Pin GPIO_PIN_7
#define HV_GPIO_Port GPIOA
#define Tube1_Pin GPIO_PIN_0
#define Tube1_GPIO_Port GPIOB
#define Tube1_EXTI_IRQn EXTI0_IRQn
#define Tube2_Pin GPIO_PIN_1
#define Tube2_GPIO_Port GPIOB
#define Tube2_EXTI_IRQn EXTI1_IRQn
#define Relay_Norm_Pin GPIO_PIN_12
#define Relay_Norm_GPIO_Port GPIOB
#define Relay_fDanger_Pin GPIO_PIN_13
#define Relay_fDanger_GPIO_Port GPIOB
#define Realy_sDanger_Pin GPIO_PIN_14
#define Realy_sDanger_GPIO_Port GPIOB
#define RS_CTR_Pin GPIO_PIN_8
#define RS_CTR_GPIO_Port GPIOA
#define Boot_Pin GPIO_PIN_11
#define Boot_GPIO_Port GPIOA
#define LED_Tick_Pin GPIO_PIN_3
#define LED_Tick_GPIO_Port GPIOB
#define LED_OK_Pin GPIO_PIN_4
#define LED_OK_GPIO_Port GPIOB
#define LED_RX_Pin GPIO_PIN_5
#define LED_RX_GPIO_Port GPIOB
#define LED_TX_Pin GPIO_PIN_6
#define LED_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
