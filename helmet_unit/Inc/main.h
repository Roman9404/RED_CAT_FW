/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#define RED_Pin GPIO_PIN_13
#define RED_GPIO_Port GPIOC
#define GREEN_Pin GPIO_PIN_14
#define GREEN_GPIO_Port GPIOC
#define BLUE_Pin GPIO_PIN_15
#define BLUE_GPIO_Port GPIOC
#define ON_BTN_Pin GPIO_PIN_1
#define ON_BTN_GPIO_Port GPIOA
#define ON_BTN_EXTI_IRQn EXTI1_IRQn
#define CONNECT_BTN_Pin GPIO_PIN_2
#define CONNECT_BTN_GPIO_Port GPIOA
#define CONNECT_BTN_EXTI_IRQn EXTI2_IRQn
#define IRQ_Pin GPIO_PIN_3
#define IRQ_GPIO_Port GPIOA
#define ADC1_IN8_BAT_Pin GPIO_PIN_0
#define ADC1_IN8_BAT_GPIO_Port GPIOB
#define ADC2_IN9_LIGHT_Pin GPIO_PIN_1
#define ADC2_IN9_LIGHT_GPIO_Port GPIOB
#define BAT_CHECK_Pin GPIO_PIN_2
#define BAT_CHECK_GPIO_Port GPIOB
#define LEFT_Pin GPIO_PIN_10
#define LEFT_GPIO_Port GPIOB
#define RIGHT_Pin GPIO_PIN_11
#define RIGHT_GPIO_Port GPIOB
#define LED_4_Pin GPIO_PIN_12
#define LED_4_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_13
#define LED_3_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_14
#define LED_2_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_15
#define LED_1_GPIO_Port GPIOB
#define EN_3_3V_Pin GPIO_PIN_10
#define EN_3_3V_GPIO_Port GPIOA
#define EN_12V_Pin GPIO_PIN_11
#define EN_12V_GPIO_Port GPIOA
#define STOP_Pin GPIO_PIN_15
#define STOP_GPIO_Port GPIOA
#define GABARIT_Pin GPIO_PIN_3
#define GABARIT_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_5
#define CE_GPIO_Port GPIOB
#define CSN_Pin GPIO_PIN_6
#define CSN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
