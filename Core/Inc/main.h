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
#include "stm32g0xx_hal.h"

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
#define TEST_LED_Pin                      GPIO_PIN_1
#define TEST_LED_GPIO_Port                GPIOA

#define WIFI_TX_Pin GPIO_PIN_2
#define WIFI_TX_GPIO_Port GPIOA

#define WIFI_RX_Pin GPIO_PIN_3
#define WIFI_RX_GPIO_Port GPIOA

#define BUZZER_Pin                GPIO_PIN_1 
#define BUZZER_GPIO_Port          GPIOB


#define PLASMA_Pin                  GPIO_PIN_12
#define PLASMA_GPIO_Port            GPIOB

//#define FAN_1_Pin GPIO_PIN_6
//#define FAN_1_GPIO_Port GPIOA

#define FAN_2_Pin 							GPIO_PIN_7
#define FAN_2_GPIO_Port 					GPIOA

#define RELAY_Pin 							GPIO_PIN_0
#define RELAY_GPIO_Port 					GPIOB

#define TEMP_Pin 							GPIO_PIN_13
#define TEMP_GPIO_Port 						GPIOB

#define UL_Pin 								GPIO_PIN_8
#define UL_GPIO_Port 						GPIOA

#define DISP_TX_Pin 						GPIO_PIN_9
#define DISP_TX_GPIO_Port 					GPIOA

//#define BUZZER_Pin GPIO_PIN_7
//#define BUZZER_GPIO_Port GPIOC

#define DISP_RX_Pin                 		GPIO_PIN_10
#define DISP_RX_GPIO_Port           		GPIOA

#define WIFI_EN_Pin                   		GPIO_PIN_12
#define WIFI_EN_GPIO_Port             		GPIOA

#define WIFI_CONFIG_Pin                   	GPIO_PIN_15
#define WIFI_CONFIG_GPIO_Port             	GPIOA
	
#define WIFI_RESET_Pin                    	GPIO_PIN_0
#define WIFI_RESET_GPIO_Port              	GPIOD


#define PTC_ADC_Pin                         GPIO_PIN_1
#define PTC_ADC_GPIO_Port                   GPIOA

#define FAN_DET_Pin                       	GPIO_PIN_0
#define FAN_DET_GPIO_Port                 	GPIOA

//RS485 RXEN AND TXEN  
#define  RS485_TXEN_Pin                    GPIO_PIN_10
#define  RS485_TXEN_GPIO_Port              GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
