#include "bsp.h"


void bsp_GetCpuID(void)
{
/* 检测CPU ID */
	{
		uint32_t CPU_Sn0, CPU_Sn1, CPU_Sn2;
		
		CPU_Sn0 = *(__IO uint32_t*)(UID_BASE); //(0x1FFF7590);
		CPU_Sn1 = *(__IO uint32_t*)(UID_BASE+ 4);
		CPU_Sn2 = *(__IO uint32_t*)(UID_BASE + 8);

		printf("CPU : STM32H743XIH6, BGA240, 主频: %dMHz\r\n", SystemCoreClock / 1000000);
		printf("UID = %08X %08X %08X\r\n", CPU_Sn2, CPU_Sn1, CPU_Sn0);
	}
}

/*
*********************************************************************************************************
*	函 数 名: Error_Handler
*	形    参: file : 源代码文件名称。关键字 __FILE__ 表示源代码文件名。
*			  line ：代码行号。关键字 __LINE__ 表示源代码行号
*	返 回 值: 无
*		Error_Handler(__FILE__, __LINE__);
*********************************************************************************************************
*/
void Error_Modbus_Handler(char *file, uint32_t line)
{
	/* 
		用户可以添加自己的代码报告源代码文件名和代码行号，比如将错误文件和行号打印到串口
		printf("Wrong parameters value: file %s on line %d\r\n", file, line) 
	*/
	
	/* 这是一个死循环，断言失败时程序会在此处死机，以便于用户查错 */
	if (line == 0)
	{
		return;
	}
	
	while(1)
	{
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_Idle
*	功能说明: 空闲时执行的函数。一般主程序在for和while循环程序体中需要插入 CPU_IDLE() 宏来调用本函数。
*			 本函数缺省为空操作。用户可以添加喂狗、设置CPU进入休眠模式的功能。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_Idle(void)
{
	/* --- 喂狗 */

	
	/* --- 让CPU进入休眠，由Systick定时中断唤醒或者其他中断唤醒 */

	/* 例如 emWin 图形库，可以插入图形库需要的轮询函数 */
	//GUI_Exec();

	/* 例如 uIP 协议，可以插入uip轮询函数 */
	//TOUCH_CapScan();
	MODH_Poll();
}

