#ifndef __FAN_H__
#define __FAN_H__
#include "main.h"


#define FAN_CCW       GPIO_PIN_6
#define FAN_CW        GPIO_PIN_7
#define FAN_GPIO      GPIOA

#define FAN_CCW_SetHigh()            HAL_GPIO_WritePin(FAN_GPIO,FAN_CCW,GPIO_PIN_SET)    // output high level
#define FAN_CCW_SetLow()             HAL_GPIO_WritePin(FAN_GPIO,FAN_CCW,GPIO_PIN_RESET)    // output low level

#define FAN_CW_SetHigh()            HAL_GPIO_WritePin(FAN_GPIO,FAN_CW,GPIO_PIN_SET)    // output high level
#define FAN_CW_SetLow()             HAL_GPIO_WritePin(FAN_GPIO,FAN_CW,GPIO_PIN_RESET)    // output low level



#define PTC_SetHigh()            HAL_GPIO_WritePin(RELAY_GPIO_Port,RELAY_Pin,GPIO_PIN_SET)    // output high level
#define PTC_SetLow()             HAL_GPIO_WritePin(RELAY_GPIO_Port,RELAY_Pin ,GPIO_PIN_RESET)    // output low level


#define PLASMA_SetHigh()            HAL_GPIO_WritePin(PLASMA_GPIO_Port,PLASMA_Pin,GPIO_PIN_SET)    // output high level
#define PLASMA_SetLow()             HAL_GPIO_WritePin(PLASMA_GPIO_Port,PLASMA_Pin,GPIO_PIN_RESET)    // output low level


//#define FAN_DETECT_ERROR()         HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)



void FAN_Stop(void);
void SterIlization(uint8_t sel);

void ShutDown_AllFunction(void);
void Dry_Function(uint8_t sel);

void Fan_CCW_Run_Max(void);
void Fan_CCW_Run_Min(void);

void Fan_Run_Fun(void);




#endif 
