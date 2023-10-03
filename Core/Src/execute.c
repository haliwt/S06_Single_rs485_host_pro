#include "execute.h"
#include "dht11.h"
#include "fan.h"
#include "tim.h"
#include "cmd_link.h"
#include "run.h"
#include "wifi_fun.h"
#include "esp8266.h"
#include "buzzer.h"



void SetPowerOn_ForDoing(void)
{

  
   run_t.gPower_On=POWER_ON;
	
 
	
    run_t.gDry = 1;
	run_t.gPlasma =1;       //"杀菌"
	run_t.gUltrasonic = 1; // "驱虫"
	run_t.gFan_level = fan_speed_max;
	
    
 }

void SetPowerOff_ForDoing(void)
{

   
	
	run_t.gPower_On = POWER_OFF;

 
   
    run_t.gDry = 0;
	run_t.gPlasma =0;       //"杀菌"
	run_t.gUltrasonic = 0; // "驱虫"
	
	
	PLASMA_SetLow(); //
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);//ultrasnoic Off 
	PTC_SetLow();
	 //input ac power be detect fan default 
	FAN_Stop();

}



void ActionEvent_Handler(void)
{
   if(run_t.ptc_warning ==0){
	if(run_t.gDry == 1){
	
        PTC_SetHigh();
	
	}
	else{
	   PTC_SetLow();
	  
	}
   }
    
	//kill
	if(run_t.gPlasma == 1){
	     PLASMA_SetHigh();
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//ultrasnoic ON 
		 
	}
	else{

		PLASMA_SetLow();
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);//ultrasnoic off
		
	}

   if(run_t.ultrasonic ==1){

      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//ultrasnoic ON 


   }
   else{

       HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);//ultrasnoic off

   }

   if(run_t.fan_warning ==0){
     Fan_Run_Fun();
  }
			
}

