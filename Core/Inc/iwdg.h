#ifndef __IWDG_H_
#define __IWDG_H_
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_IWDG_Init(void);

/* USER CODE BEGIN Prototypes */
void iwdg_feed(void);

/* USER CODE END Prototypes */





#endif 
