#include "iwdg.h"
#include "bsp.h"
/***********************************************************
*
* Tout =(4*2^prer *rlr)/32 (ms) //IWDG ->LSI ->32KHz
*	   =((4*2^0)*4095)/32
*	   = 511.8ms.
*
* Tout = (64 * 4095) /32 = 8s.
*
* If no LICENSE file comes with this software, it is provided AS-IS.
*
*****************************************************************/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;// 64*4095/32KH = 8190ms= 8.190s
  hiwdg.Init.Window = 4095;  //12bit of max value 
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    //Error_Handler();
  }
}


void iwdg_feed(void)
{
   if(run_t.gTimer_dogfood_times > 5){
	   run_t.gTimer_dogfood_times =0;
       HAL_IWDG_Refresh(&hiwdg);

    }
}

/* USER CODE END IWDG_Init 2 */





