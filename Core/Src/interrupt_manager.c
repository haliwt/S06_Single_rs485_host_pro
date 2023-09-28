#include "interrupt_manager.h"
#include "run.h"
#include "esp8266.h"
#include "wifi_fun.h"
#include "cmd_link.h"
#include "usart.h"


//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//   uint32_t temp;
//
//    if(huart->Instance==USART1){
//	
//		if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_ORE)!=RESET){
//
//		__HAL_UART_CLEAR_OREFLAG(&huart1);
//		UART_Start_Receive_IT(&huart1,inputBuf,1);
//
//		}
//		__HAL_UNLOCK(&huart1);
//        __HAL_UART_CLEAR_OREFLAG(&huart1);
//		  
//		temp = USART1->RDR;
//		UART_Start_Receive_IT(&huart1,inputBuf,1);
//	
//		}
//
//
//
//
//}

void USART2_Receive_Interrupt_Data(void)
{
	

}
/*****************************************************************************************
	*
	*Function Name:static void Wifi_ReceiveData_Handler(void)
	*Function :receive from wifi usart of data 
	*Input Ref: NO
	*Return Ref:NO
	*
*
******************************************************************************************/
void USART2_WIFI_Receive_Data(void)
{

  
  
}

/**
  * Function Name: void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  * Function: Tim3 interrupt call back function
  * Tim3 timer :timing time 10ms
  * 
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    static uint8_t tm0;
    if(htim->Instance==TIM14){
		
	   tm0 ++ ;
       run_t.gTimer_senddata_panel++;
	 if(tm0 > 99){//100ms *10 = 1000ms =1s
        tm0 =0;
		
	    run_t.gTimer_continuce_works_time++;
		run_t.gFan_counter++;

        run_t.gTimer_1s ++;
	    run_t.gTimer_10s++;
		
     
	
	   run_t.gTimer_send_cloud_state++;
	     run_t.gTimer_usart_error_2++;


	   //usart 1
	    run_t.gTimer_check_iwdg_flag++;
	   run_t.gTimer_usart_error++;

	    run_t.gTimer_ptc_adc_times ++; 
        run_t.gTimer_fan_adc_times++ ;
		run_t.gTimer_rs485_times++;
	 
		
	 

	  
	  
	   
	 
	}
 	}
 }
