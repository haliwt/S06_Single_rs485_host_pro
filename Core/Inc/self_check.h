#ifndef __SELF_CHECK_H
#define __SELF_CHECK_H
#include "main.h"



void MainBoard_Self_Inspection_PowerOn_Fun(void);


extern void (*Self_CheckFan_Handler)(uint32_t channel,uint8_t times);

///extern void (*EUSART_TxDefaultInterruptHandler)(void);
//extern void (*EUSART_RxDefaultInterruptHandler)(void);

void Self_Check_Fan_Handler(void (* fanHandler)(uint32_t channel,uint8_t times));



#endif 
