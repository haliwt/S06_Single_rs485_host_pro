#include "self_check.h"
#include "run.h"
#include "esp8266.h"
#include "publish.h"
#include "cmd_link.h"
#include "wifi_fun.h"
#include "adc.h"
#include "buzzer.h"
#include "fan.h"

void (*Self_CheckFan_Handler)(uint32_t channel,uint8_t times);



/**********************************************************************
	*
	*Functin Name: void MainBoard_Itself_PowerOn_Fun(void)
	*Function :
	*Input Ref:  key of value
	*Return Ref: NO
	*
**********************************************************************/
void Self_Check_Fain_Init(void)
{

  Self_Check_Fan_Handler(Get_Fan_Adc_Fun);//


}


/**********************************************************************
	*
	*Functin Name: void MainBoard_Itself_PowerOn_Fun(void)
	*Function :
	*Input Ref:  key of value
	*Return Ref: NO
	*
**********************************************************************/


/**************************************************************
	*
	*Function Name:static void Self_Checking_Function(void)
	*
	*
	*
**************************************************************/
void Self_Check_Fan_Handler(void (* fanHandler)(uint32_t channel,uint8_t times))
{
   Self_CheckFan_Handler = fanHandler;

}




