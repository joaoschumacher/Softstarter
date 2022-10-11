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
#include "stm32f4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define DETECT_Pin GPIO_PIN_0
#define DETECT_GPIO_Port GPIOC
#define OUT_TO_GATE_Pin GPIO_PIN_1
#define OUT_TO_GATE_GPIO_Port GPIOC
#define INHIBIT_Pin GPIO_PIN_2
#define INHIBIT_GPIO_Port GPIOC
#define BYPASS_Pin GPIO_PIN_3
#define BYPASS_GPIO_Port GPIOC
#define POT_Pin GPIO_PIN_0
#define POT_GPIO_Port GPIOA
#define BOTAO3_Pin GPIO_PIN_1
#define BOTAO3_GPIO_Port GPIOA
#define BOTAO3_EXTI_IRQn EXTI1_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define BOTAO2_Pin GPIO_PIN_4
#define BOTAO2_GPIO_Port GPIOA
#define BOTAO2_EXTI_IRQn EXTI4_IRQn
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define D7_Pin GPIO_PIN_6
#define D7_GPIO_Port GPIOA
#define D6_Pin GPIO_PIN_7
#define D6_GPIO_Port GPIOA
#define REDE_Pin GPIO_PIN_4
#define REDE_GPIO_Port GPIOC
#define BOTAO1_Pin GPIO_PIN_0
#define BOTAO1_GPIO_Port GPIOB
#define BOTAO1_EXTI_IRQn EXTI0_IRQn
#define LCD_RST_Pin GPIO_PIN_10
#define LCD_RST_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_12
#define LCD_CS_GPIO_Port GPIOB
#define LCD_SCK_Pin GPIO_PIN_13
#define LCD_SCK_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_14
#define LCD_DC_GPIO_Port GPIOB
#define LCD_MO_Pin GPIO_PIN_15
#define LCD_MO_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_7
#define D4_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_4
#define LCD_EN_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_5
#define LCD_RS_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_6
#define D5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
