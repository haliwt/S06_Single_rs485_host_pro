#ifndef __BUZZER_H_
#define __BUZZER_H_
#include "main.h"



#define BUZZER_SetHigh()            HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_SET)    // output high level
#define BUZZER_SetLow()             HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_RESET)    // output low level
#define BUZZER_TOGGLE()             HAL_GPIO_TogglePin(BUZZER_GPIO_Port,BUZZER_Pin ) 


void Buzzer_KeySound(void);



#endif 
