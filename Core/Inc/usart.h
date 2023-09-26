/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define USART_WIFI     USART2
#define USART_CMD      USART1
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
//#define RS485_RX_EN()	 HAL_GPIO_WritePin(GPIOB,RS485_TXEN_Pin,GPIO_PIN_RESET)//RS485_TXEN_GPIO_PORT->BSRR = ((uint32_t)RS485_TXEN_PIN << 16U)
//#define RS485_TX_EN()	 HAL_GPIO_WritePin(GPIOB,RS485_TXEN_Pin,GPIO_PIN_SET)//RS485_TXEN_GPIO_PORT->BSRR = RS485_TXEN_PIN

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

