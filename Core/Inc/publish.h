#ifndef __PUBLISH_H_
#define __PUBLISH_H_
#include "main.h"

void Init_Publisher_Data_Ref(void);
//send one byte 
void MYUSART_SendData(uint8_t  data);



void Publish_Data_ProdKey(void);
void Publish_Command_SmartCofnig(void);
void Publish_Command_Query(void);
void Publish_Return_Repeat_Data(void);


void Publish_Power_ON_State(void);

void Publish_Power_OFF_State(void);





void Publish_Reference_Update_State(void);

#endif 


