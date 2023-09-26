#include "fan.h"
#include "main.h"
#include "tim.h"
#include "run.h"
#include "delay.h"
#include "adc.h"




void Fan_CCW_Run_Max(void)
{
   FAN_CW_SetLow();
   MX_TIM3_Init(23,9,10); //100KHz
   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  
}

void Fan_CCW_Run_Min(void)
{
    FAN_CW_SetLow();
    MX_TIM3_Init(23,99,50); //10KHz
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);



}

 
void FAN_Stop(void)
{
    FAN_CCW_SetLow(); //brake
   // FAN_CCW_SetLow(); //PA6
   HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
    
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


