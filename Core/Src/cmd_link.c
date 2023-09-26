#include "cmd_link.h"
#include <string.h>
#include "usart.h"
#include "run.h"
#include "fan.h"
#include "interrupt_manager.h"


#define MAX_BUFFER_SIZE  30




uint8_t  inputBuf[5];
uint8_t  inputCmd[5];
uint8_t  wifiInputBuf[1];

uint32_t temp;


static uint8_t transferSize;
static uint8_t outputBuf[MAX_BUFFER_SIZE];

volatile uint8_t transOngoingFlag;
volatile uint8_t usart2_transOngoingFlag;


/********************************************************************************
	**
	*Function Name:void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	*Function :UART callback function  for UART interrupt for receive data
	*Input Ref: structure UART_HandleTypeDef pointer
	*Return Ref:NO
	*
*******************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t state;
 

	if(huart->Instance==USART1)//if(huart==&huart1) // Motor Board receive data (filter)
	{
      
		switch(state)
		{
		case 0:  //#0
			if(inputBuf[0] == 'T')  //hex :54 - "T" -fixed
				state=1; //=1
            else
                state=0;
		break;
		case 1: //#1
             if(inputBuf[0] == 'K')  {//hex :4B - "K" -fixed
               
				state=2; //=1

             }
			else{
			   state =0;
			}
			break;
            
        case 2:
			 if(inputBuf[0] == 'P' || inputBuf[0] == 'C' ||inputBuf[0] == 'Z'){
			    inputCmd[0]= inputBuf[0];
				 state =3;

			 }
			 else
                 state =0;
        
        break;
        
        case 3:
		    
	        inputCmd[1]= inputBuf[0];
	       
		     run_t.decodeFlag =1;
			 
	         state = 0;
        
        break;

	
			
		}
	 // __HAL_UART_CLEAR_OREFLAG(&huart1);
		HAL_UART_Receive_IT(&huart1,inputBuf,1);//UART receive data interrupt 1 byte
		
	 }
    
  
 }

/********************************************
	*
	*Function Name:void Display_Decode_Function(void)
    *Function: receive dsipay panel of order
    *Input Ref:NO
    *Return Ref:NO

*********************************************/ 
void Display_Decode_Function(void)
{
   if(run_t.decodeFlag==1){
   
      run_t.decodeFlag =0;
	  run_t.process_run_guarantee_flag =1;
      Decode_RunCmd();
   
      
     }
}
/********************************************
	*
	*Function Name:void Display_Decode_Function(void)
    *Function: receive dsipay panel of order
    *Input Ref:NO
    *Return Ref:NO

*********************************************/ 
void USART1_Cmd_Error_Handler(UART_HandleTypeDef *huart)
{
   
  if(huart->Instance==USART1){
    

	  if(run_t.gTimer_usart_error >5){
	  	run_t.gTimer_usart_error=0;

        __HAL_UART_CLEAR_OREFLAG(&huart1);
		
        
        //  temp = USART1->ISR;
          temp = USART1->RDR;
  
		    
          
	       UART_Start_Receive_IT(&huart1,inputBuf,1);
          
         
          
      }
  }
         
}
//void USART2_Cmd_Error_Handler(UART_HandleTypeDef *huart)
//{
//   uint32_t temp;
//  if(huart->Instance==USART2){
//    
//
//	  if(run_t.gTimer_usart_error_2 >37){
//	  	run_t.gTimer_usart_error_2=0;
//
//          __HAL_UART_CLEAR_OREFLAG(&huart2);
//          temp= USART2->RDR;
//		    
//          
//	   }
//	  }
//         
//}
/********************************************************************************
	**
	*Function Name:sendData_Real_TimeHum(uint8_t hum,uint8_t temp)
	*Function :
	*Input Ref: humidity value and temperature value
	*Return Ref:NO
	*
*******************************************************************************/
void sendData_Reference_Data(uint8_t dry,uint8_t kill,uint8_t mouse)
{

	//crc=0x55;
	outputBuf[0]='M'; //master
	outputBuf[1]='A'; //41
	outputBuf[2]='R'; //
	outputBuf[3]=dry; //	
	outputBuf[4]=kill; // // one command parameter
	outputBuf[5]=mouse;
	
	//for(i=3;i<6;i++) crc ^= outputBuf[i];
	//outputBuf[i]=crc;
	transferSize=6;
	if(transferSize)
	{
		while(transOngoingFlag); //UART interrupt transmit flag ,disable one more send data.
		transOngoingFlag=1;
		HAL_UART_Transmit_IT(&huart1,outputBuf,transferSize);
	}



}
/********************************************************************************
	**
	*Function Name:sendData_Real_TimeHum(uint8_t hum,uint8_t temp)
	*Function :
	*Input Ref: humidity value and temperature value
	*Return Ref:NO
	*
*******************************************************************************/
void sendData_Real_TimeHum(uint8_t hum,uint8_t temp)
{

	//crc=0x55;
	outputBuf[0]='M'; //master
	outputBuf[1]='A'; //41
	outputBuf[2]='D'; //44	// 'D' data
	outputBuf[3]=hum; //	// 'R' rotator motor for select filter
	outputBuf[4]=temp; // // one command parameter
	
	//for(i=3;i<6;i++) crc ^= outputBuf[i];
	//outputBuf[i]=crc;
	transferSize=5;
	if(transferSize)
	{
		while(transOngoingFlag); //UART interrupt transmit flag ,disable one more send data.
		transOngoingFlag=1;
		HAL_UART_Transmit_IT(&huart1,outputBuf,transferSize);
	}



}
void SendWifiData_To_PanelTime(uint8_t dat1)
{
   
	
		outputBuf[0]='M'; //4D
		outputBuf[1]='A'; //41
		outputBuf[2]='T'; //44	// 'T' time
		outputBuf[3]=dat1; //	
		
		transferSize=4;
		if(transferSize)
		{
			while(transOngoingFlag); //UART interrupt transmit flag ,disable one more send data.
			transOngoingFlag=1;
			HAL_UART_Transmit_IT(&huart1,outputBuf,transferSize);
		}



}
/********************************************************************************
**
*Function Name:void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
*Function :UART callback function  for UART interrupt for transmit data
*Input Ref: structure UART_HandleTypeDef pointer
*Return Ref:NO
*
*******************************************************************************/
void SendWifiData_To_PanelTemp(uint8_t dat1)
{
   
	//crc=0x55;
		outputBuf[0]='M'; //4D
		outputBuf[1]='A'; //41
		outputBuf[2]='P'; // 'T' time
		outputBuf[3]=dat1; //	
	
		
		//for(i=3;i<6;i++) crc ^= outputBuf[i];
		//outputBuf[i]=crc;
		transferSize=4;
		if(transferSize)
		{
			while(transOngoingFlag); //UART interrupt transmit flag ,disable one more send data.
			transOngoingFlag=1;
			HAL_UART_Transmit_IT(&huart1,outputBuf,transferSize);
		}

}

/********************************************************************************
**
*Function Name:void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
*Function :UART callback function  for UART interrupt for transmit data
*Input Ref: structure UART_HandleTypeDef pointer
*Return Ref:NO
*
*******************************************************************************/
void SendWifiCmd_To_Order(uint8_t odata)
{
  

			outputBuf[0]='M'; //mainboard
			outputBuf[1]='A'; //A ->
			outputBuf[2]='E'; //'C' ->control 
			outputBuf[3]=odata; //	
		
			transferSize=4;
			if(transferSize)
			{
				while(transOngoingFlag); //UART interrupt transmit flag ,disable one more send data.
				transOngoingFlag=1;
				HAL_UART_Transmit_IT(&huart1,outputBuf,transferSize);
			}



}
#if 0
void SendData_Real_GMT(uint8_t hdata,uint8_t mdata,uint8_t sdata)
{
   

	outputBuf[0]='M'; //4D
	outputBuf[1]='A'; //41
	outputBuf[2]='B'; //44	// 'C' ->control 
	outputBuf[3]=hdata; //	
	outputBuf[4]=mdata; //
	outputBuf[5] =sdata;
	//for(i=3;i<6;i++) crc ^= outputBuf[i];
	//outputBuf[i]=crc;
	transferSize=6;
	if(transferSize)
	{
		while(transOngoingFlag); //UART interrupt transmit flag ,disable one more send data.
		transOngoingFlag=1;
		HAL_UART_Transmit_IT(&huart1,outputBuf,transferSize);
	}

}
#endif 
/********************************************************************************
	**
	*Function Name:void Answering_Signal_USART1_Handler(void)
	*Function :UART1 acknowleg 
	*Input Ref: cmddata =0,command ,cmddata =1,data
	*Return Ref:NO
	*
*******************************************************************************/
void Answering_Signal_USART1_Handler(uint8_t cmdordata,uint8_t data)
{
    outputBuf[0]=0x55; //head :0x55
	outputBuf[1]=cmdordata; //ID:command or data
	outputBuf[2]=0; //Type: 0 ->uint8_t 
	outputBuf[3]=0x01; //length: 1 	
	outputBuf[4]=data; //
	outputBuf[5] = outputBuf[0]+outputBuf[1]+outputBuf[2]+outputBuf[3]+outputBuf[4];
	//for(i=3;i<6;i++) crc ^= outputBuf[i];
	//outputBuf[i]=crc;
	transferSize=6;
	if(transferSize)
	{
		while(transOngoingFlag); //UART interrupt transmit flag ,disable one more send data.
		transOngoingFlag=1;
		HAL_UART_Transmit_IT(&huart1,outputBuf,transferSize);
	}

}


/********************************************************************************
**
*Function Name:void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
*Function :UART callback function  for UART interrupt for transmit data
*Input Ref: structure UART_HandleTypeDef pointer
*Return Ref:NO
*
*******************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)
	{
		transOngoingFlag=0; //UART Transmit interrupt flag =0 ,RUN
	}

//	if(huart== &huart2){
//
//       usart2_transOngoingFlag =0;
//
//	}

}





