#include "fan.h"
#include "main.h"
#include "tim.h"
#include "run.h"
#include "delay.h"
#include "adc.h"
#include "wifi_fun.h"




void Fan_CCW_Run_Max(void)
{
   FAN_CW_SetLow();
  
  
  
}

void Fan_CCW_Run_Min(void)
{
    FAN_CW_SetLow();
   
   



}

 
void FAN_Stop(void)
{
    FAN_CCW_SetLow(); //brake
  
  
    
}
void ShutDown_AllFunction(void)
{
	
	PLASMA_SetLow(); //
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);//ultrasnoic ON 
	PTC_SetLow();
	FAN_Stop();



}




//"杀毒" 
void SterIlization(uint8_t sel)
{
    if(sel==0){//open 
		 
		 PLASMA_SetHigh();
		

	}
	else{ //close

		PLASMA_SetLow();
	

	}



}
//PTC
void Dry_Function(uint8_t sel)
{
   if(sel ==0){

  

		PTC_SetHigh();

   }
   else{

       PTC_SetLow();

   }

}


//FAN 
void Fan_Run_Fun(void)
{
   if(run_t.gFan_level==fan_speed_max){
      Fan_CCW_Run_Max();
   }
   else if(run_t.gFan_level==fan_speed_min){
      Fan_CCW_Run_Min();

   }
   else if(run_t.gFan_level==fan_speed_sotp){

        FAN_Stop();


   }


}


void POWER_KEY_INPUT_FUN(void)
{
    static uint8_t key_power;



	if(POWER_KEY_INPUT()==0){
        HAL_Delay(20);
	if(POWER_KEY_INPUT()==0){
       while(POWER_KEY_INPUT()==0);
		if(run_t.gPower_On == POWER_ON){

				key_power=1;
			}
			else{
			   key_power=5;

			}


	   }

     }

	 if(key_power==1 && POWER_KEY_INPUT()==1){
		key_power++;

	    run_t.RunCommand_Label = POWER_OFF;
		SendWifiCmd_To_Order(WIFI_POWER_OFF);
		HAL_Delay(5);


	 }
	 else if(key_power==5 && POWER_KEY_INPUT()==1){
      key_power++;
	  run_t.RunCommand_Label = POWER_ON;
	  SendWifiCmd_To_Order(WIFI_POWER_ON);
	  HAL_Delay(5);


	 }

	 if(key_power==2){
        key_power++;

	    SendWifiCmd_To_Order(WIFI_POWER_OFF);
		HAL_Delay(5);


	 }

	 if(key_power==6){

	    key_power++;
	 	
	    SendWifiCmd_To_Order(WIFI_POWER_ON);
	     HAL_Delay(5);


	 }

}


