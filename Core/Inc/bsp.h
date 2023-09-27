#ifndef __BSP_H_
#define __BSP_H_

#define STM32G030_M_1   

  
/* ����Ƿ����˿������ͺￄ1�7 */
#if !defined (STM32G030_M_1 )
	#error "Please define the board model : STM32_V7"
#endif

/* ���� BSP �汾�� */
#define __STM32H7_BSP_VERSION		"1.1"

/* CPU����ʱִ�еĺ��� */
//#define CPU_IDLE()		bsp_Idle()

/* ����ȫ���жϵĺ� */
#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */

/* ���������ڵ��Խ׶��Ŵ� */
#define BSP_Printf		printf
//#define BSP_Printf(...)

#define EXTI9_5_ISR_MOVE_OUT		/* bsp.h �ж�����У���ʾ�������Ƶￄ1�7 stam32f4xx_it.c�� �����ظ����� */

#define ERROR_HANDLER()		Error_Handler(__FILE__, __LINE__);

/* Ĭ���ǹر�״̬ */
#define  Enable_EventRecorder  0

#if Enable_EventRecorder == 1
	#include "EventRecorder.h"
#endif

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif


/* ͨ��ȡ��ע�ͻ�������ע�͵ķ�ʽ�����Ƿ�����ײ�����ģ�ￄ1�7 */

#include "modbus_host.h"
#include "bsp_user_lib.h"
#include "bsp_msg.h"
#include "bsp_uart_fifo.h"
#include "usart.h"
#include "iwdg.h"

void bsp_Init(void);
void bsp_Idle(void);

void bsp_GetCpuID(void);
void Error_Modbus_Handler(char *file, uint32_t line);






#endif 
