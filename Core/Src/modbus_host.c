#include "modbus_host.h"
#include "bsp.h"

/*
   功能码： 0x01 ->power on or off
           0x02 ->PTC open or off
		   0x03 ->Plasma open or off
		   0x04 ->ultrasonic open or off 
		   0x05 ->fan open or off 
		   0xfe -> fan is fault
		   0xff -> ptc is fault


*/
/*
*********************************************************************************************************
*	                                   变量
*********************************************************************************************************
*/
#define TIMEOUT		1000		/* 接收命令超时时间, 单位ms */
#define NUM			1			/* 循环发送次数 */

/*
Baud rate	Bit rate	 Bit time	 Character time	  3.5 character times
  2400	    2400 bits/s	  417 us	      4.6 ms	      16 ms
  4800	    4800 bits/s	  208 us	      2.3 ms	      8.0 ms
  9600	    9600 bits/s	  104 us	      1.2 ms	      4.0 ms
 19200	   19200 bits/s    52 us	      573 us	      2.0 ms
 38400	   38400 bits/s	   26 us	      286 us	      1.75 ms(1.0 ms)
 115200	   115200 bit/s	  8.7 us	       95 us	      1.75 ms(0.33 ms) 后面固定都为1750us
*/
typedef struct
{
	uint32_t Bps;
	uint32_t usTimeOut;
}MODBUSBPS_T;

const MODBUSBPS_T ModbusBaudRate[] =
{	
    {2400,	16000}, /* 波特率2400bps, 3.5字符延迟时间16000us */
	{4800,	 8000}, 
	{9600,	 4000},
	{19200,	 2000},
	{38400,	 1750},
	{115200, 1750},
	{128000, 1750},
	{230400, 1750},
};

MODH_T g_tModH = {0};

VAR_T g_tVar;
uint8_t crc16_check_flag;






/*
*********************************************************************************************************
*	                                   函数声明
*********************************************************************************************************
*/

static void MODH_RxTimeOut(void);
static void MODH_AnalyzeApp(void);



static void MODH_Read_Address_01H(void);

static void MODH_Read_02H(void);
static void MODH_Read_03H(void);
static void MODH_Read_04H(void);
static void MODH_Read_05H(void);
static void MODH_Read_06H(void);
static void MODH_Read_10H(void);

/*
*********************************************************************************************************
*	函 数 名: MODH_SendPacket
*	功能说明: 发送数据包 COM1口
*	形    参: _buf : 数据缓冲区
*			  _len : 数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_SendPacket(uint8_t *_buf, uint16_t _len)
{
	RS485_SendBuf(_buf, _len);
}

/*
*********************************************************************************************************
*	函 数 名: MODH_SendAckWithCRC
*	功能说明: 发送应答,自动加CRC.  
*	形    参: 无。
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_SendAckWithCRC(void)
{
	uint16_t crc;
	
	crc = CRC16_Modbus(g_tModH.TxBuf, g_tModH.TxCount);
	g_tModH.TxBuf[g_tModH.TxCount++] = crc >> 8;
	g_tModH.TxBuf[g_tModH.TxCount++] = crc;	
	MODH_SendPacket(g_tModH.TxBuf, g_tModH.TxCount);
}

/*
*********************************************************************************************************
*	函 数 名: MODH_AnalyzeApp
*	功能说明: 分析应用层协议。处理应答。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_AnalyzeApp(void)
{	
	
    MODH_Read_Address_01H();
	   
}
/*
*********************************************************************************************************
*	函 数 名: MODH_Send00H
*	功能说明: 发送00H指令，查询所有从机线圈寄存器 功能码 0x01
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send00H_Power_OnOff(uint8_t _addr, uint8_t _data_len,uint8_t _data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 发送地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = MasterAddr;  /* 本地地址*/
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x01;		/* 功能码 开机或者关机 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _data_len;	/* 数据长度*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _data;		/* 数据 */	
	
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck01H = 0;		/* 清接收标志 */
	//g_tModH.RegNum = _num;		/* 寄存器个数 */
	//g_tModH.Reg01H = _reg;		/* 保存01H指令中的寄存器地址，方便对应答数据进行分类 */	
}
/*
*********************************************************************************************************
*	函 数 名: MODH_Send01H
*	功能说明: 发送01H指令，查询1个或多个线圈寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send00H_Ptc_OnOff(uint8_t _addr,uint8_t _data_len,uint8_t _data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 发送地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = MasterAddr;  /* 本地地址*/
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x02;		/* 功能码 PTC加开或者关闭 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _data_len;	/* 数据长度*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _data;		/* 数据 */	
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck01H = 0;		/* 清接收标志 */
	//g_tModH.RegNum = _num;		/* 寄存器个数 */
	//g_tModH.Reg01H = _reg;		/* 保存01H指令中的寄存器地址，方便对应答数据进行分类 */	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send02H
*	功能说明: 发送02H指令，读离散输入寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send00H_Plasma_OnOff(uint8_t _addr,uint8_t _data_len,uint8_t _data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 发送地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = MasterAddr;  /* 本地地址*/
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x03;		/* 功能码 等离子开或者关闭 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _data_len;	/* 数据长度*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _data;		/* 数据 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck02H = 0;		/* 清接收标志 */
	//g_tModH.RegNum = _num;		/* 寄存器个数 */
	//g_tModH.Reg02H = _reg;		/* 保存02H指令中的寄存器地址，方便对应答数据进行分类 */	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send03H
*	功能说明: 发送03H指令，打开或者关闭超声波
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send00H_Ultrasonic_OnOff(uint8_t _addr,uint8_t _data_len,uint8_t _data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 发送地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = MasterAddr;  /* 本地地址*/
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x04;		/* 功能码 超声波开或者关闭 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _data_len;	/* 数据长度*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _data;		/* 数据 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck03H = 0;		/* 清接收标志 */
	//g_tModH.RegNum = _num;		/* 寄存器个数 */
	//g_tModH.Reg03H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send04H
*	功能说明: 发送04H指令，读输入寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send00H_Fan_OnOff(uint8_t _addr,uint8_t _data_len,uint8_t _data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 发送地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = MasterAddr;  /* 本地地址*/
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x05;		/* 功能码 风扇开或者关闭 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _data_len;	/* 数据长度*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _data;		/* 数据 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck04H = 0;		/* 清接收标志 */
	//g_tModH.RegNum = _num;		/* 寄存器个数 */
	//g_tModH.Reg04H = _reg;		/* 保存04H指令中的寄存器地址，方便对应答数据进行分类 */	
}
/*
*********************************************************************************************************
*	函 数 名: MODH_RxTimeOut
*	功能说明: 超过3.5个字符时间后执行本函数。 设置全局变量 g_rtu_timeout = 1; 通知主程序开始解码。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_RxTimeOut(void)
{
	//g_modh_timeout = 1;
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Poll
*	功能说明: 接收控制器指令. 1ms 响应时间。
*	形    参: 无
*	返 回 值: 0 表示无数据 1表示收到正确命令
*********************************************************************************************************
*/
void MODH_Poll(void)
{	
	uint16_t crc1 ;
	
#if 0	
	if (g_modh_timeout == 0)	/* 超过3.5个字符时间后执行MODH_RxTimeOut()函数。全局变量 g_rtu_timeout = 1 */
	{
		/* 没有超时，继续接收。不要清零 g_tModH.RxCount */
		return ;
	}

	/* 收到命令
		05 06 00 88 04 57 3B70 (8 字节)
			05    :  数码管屏的号站，
			06    :  指令
			00 88 :  数码管屏的显示寄存器
			04 57 :  数据,,,转换成 10 进制是 1111.高位在前,
			3B70  :  二个字节 CRC 码	从05到 57的校验
	*/
	g_modh_timeout = 0;
    HAL_UART_Receive_DMA(&huart2,g_tModH.RxBuf, 0x07);
	/* 接收到的数据小于4个字节就认为错误，地址（8bit）+指令（8bit）+操作寄存器（16bit） */
	/* 发送地址+本地地址+功能码+数据长度+数据+CRC16(2BYTE)*/
	if (g_tModH.RxCount < 5)
	{
		goto err_ret;
	}
#endif 
    if(g_tModH.Rx_rs485_data_flag == rx_rs485_data_success){
		/* 计算CRC校验和，这里是将接收到的数据包含CRC16值一起做CRC16，结果是0，表示正确接收 */
		crc1 = CRC16_Modbus(g_tModH.RxBuf,g_tModH.RxCount);
		if (crc1 != 0)
		{
			g_tModH.RxCount = 0;	/* 必须清零计数器，方便下次帧同步 */
			g_tModH.Rx_rs485_data_flag=0;
			Answerback_RS485_Signal(g_tModH.RxBuf[1],0xff,g_tModH.RxBuf[3],0x01);
		}
		else{
        	crc16_check_flag = 1;
			g_tModH.Rx_rs485_data_flag=0;
		    g_tModH.RxCount =0;
		    

		}
    }
	if(crc16_check_flag==1){
		/* 分析应用层协议 */
		MODH_AnalyzeApp();
		crc16_check_flag=0;
	 
	
   	}

	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_01H
*	功能说明: 分析01H指令的应答数据，读取线圈状态，bit访问
*             [发送的地址]+[本机地址]+[功能码]+[数据长度]+[数据]+[CRC16低]+[CRC16高]
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_Address_01H(void)
{
	
	uint8_t bytes_zero,byte_load_addr,byte_fun_code,byte_len,byte_data,fun_byte;
	  
	   bytes_zero = g_tModH.RxBuf[0];	/* 主机  地址   0x01 */
	   byte_load_addr = g_tModH.RxBuf[1]; /* 从机地址，0x*/
	   byte_fun_code = g_tModH.RxBuf[2];
	   byte_len = g_tModH.RxBuf[3];
	   byte_data = g_tModH.RxBuf[4];

	   Answerback_RS485_Signal(byte_load_addr,byte_fun_code,byte_len,byte_data);
	   
	   
	   if(bytes_zero == MasterAddr){
		switch (byte_fun_code)
		{
			case mod_power: //0x0101
				
				switch(byte_data){

                   case 0:
                       run_t.RunCommand_Label= POWER_OFF;
				      
				       SendWifiCmd_To_Order(WIFI_POWER_OFF);
				      
				   break;

				   case 1:
				      run_t.RunCommand_Label= POWER_ON;
					  SendWifiCmd_To_Order(WIFI_POWER_ON);

				   break;

				}	
					
				g_tModH.fAck01H = 1;
				
			break;

			case mod_ptc:

			   if(run_t.gPower_On == POWER_ON){
			  
			   switch(byte_data){

                   case 0:
                      run_t.gDry = 0;
				   break;

				   case 1:
				      
					run_t.gDry = 1;
				   break;

				}	
                g_tModH.fAck02H = 1;
			   }
			break;

			case mod_plasma:

				 if(run_t.gPower_On == POWER_ON){
			   
			     switch(byte_data){

                   case 0:
                     run_t.gPlasma=0; 
				   break;

				   case 1:
				      
				    run_t.gPlasma=1;

				   break;

				}	
                g_tModH.fAck03H = 1;
				}

			break;

			case mod_ulrasonic:

			    if(run_t.gPower_On == POWER_ON){
				
				 switch(byte_data){

                   case 0:
                       run_t.ultrasonic = 0;
				   break;

				   case 1:
				     run_t.ultrasonic = 1;

				   break;

				}	
                g_tModH.fAck04H = 1;

			   }

			break;

			case mod_fan:

			break;
	    }
		
	}
}
/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_01H
*	功能说明: 单个参数. 通过发送01H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_Power_01H(uint8_t add,uint8_t _num,uint8_t _reg)
{
	//int32_t time1,time2;
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		//MODH_Send01H (SlaveAddr_1, _reg, _num);		  /* 发送命令 */
		MODH_Send00H_Power_OnOff(add,_num,_reg);
		//time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
		run_t.gTimer_rs485_times=0;
		//while (1)				
		{
			bsp_Idle();
           // time2 = bsp_CheckRunTime(time1);
			if(run_t.gTimer_rs485_times> 2) ///* 等待应答,超时或接收到应答则break  */
            {
				break;		/* 通信超时了 */
			}
			
			if (g_tModH.fAck01H > 0)
			{
				break;		/* 接收到应答 */
			}
		}
		
		if (g_tModH.fAck01H > 0)
		{
			break;			/* 循环NUM次，如果接收到命令则break循环 */
		}
	}
	
	if (g_tModH.fAck01H == 0)
	{
		return 0;
	}
	else 
	{
		return 1;	/* 01H 读成功 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_02H
*	功能说明: 单个参数. 通过发送02H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_PTC_02H(uint8_t add,uint8_t _num,uint8_t _reg)
{
	
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		//MODH_Send02H (SlaveAddr_1, _reg, _num);
		MODH_Send00H_Ptc_OnOff(add,_num,_reg);
	    //time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
		//while (1)
		{
			bsp_Idle();

			//if(bsp_CheckRunTime(time1) > TIMEOUT)//if(run_t.gTimer_rs485_times> TIMEOUT)		//if (bsp_CheckRunTime(time1) > TIMEOUT)		
			if(run_t.gTimer_rs485_times> TIMEOUT)
			{
				break;		/* 通信超时了 */
			}
			
			if (g_tModH.fAck02H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck02H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck02H == 0)
	{
		return 0;
	}
	else 
	{
		return 1;	/* 02H 读成功 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_03H
*	功能说明: 单个参数. 通过发送03H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_Plasma_03H(uint8_t add,uint8_t _num,uint8_t _reg)
{
	int32_t time1;
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		//MODH_Send03H (SlaveAddr_1, _reg, _num);
		MODH_Send00H_Plasma_OnOff(add,_num,_reg);
		//time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
			run_t.gTimer_rs485_times=0;
		//while (1)
		{
			bsp_Idle();

			//if (bsp_CheckRunTime(time1) > TIMEOUT)//
			if(run_t.gTimer_rs485_times> TIMEOUT)		
			{
				break;		/* 通信超时了 */
			}
			
			if (g_tModH.fAck03H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck03H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck03H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else 
	{
		return 1;	/* 写入03H参数成功 */
	}
}


/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_04H
*	功能说明: 单个参数. 通过发送04H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_Ultrasonic_04H(uint8_t add,uint8_t _num,uint8_t _reg)
{
	int32_t time1;
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		//MODH_Send04H (SlaveAddr_1, _reg, _num);
		MODH_Send00H_Ultrasonic_OnOff(add,_num,_reg);
		//time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
			run_t.gTimer_rs485_times=0;
		//while (1)
		{
			bsp_Idle();

			//if(bsp_CheckRunTime(time1) > TIMEOUT)//
			if(run_t.gTimer_rs485_times> TIMEOUT)		//if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;		/* 通信超时了 */
			}
			
			if (g_tModH.fAck04H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck04H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck04H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else 
	{
		return 1;	/* 04H 读成功 */
	}
}

/********************************************************************************
	**
	*Function Name:
	*Function :UART callback function  for UART interrupt for transmit data
	*Input Ref: structure UART_HandleTypeDef pointer
	*Return Ref:NO
	*
*******************************************************************************/
void Answerback_RS485_Signal(uint8_t addr,uint8_t fun_code,uint8_t len,uint8_t data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = MasterAddr;		/* 从站地址 发送地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = addr;  /* 本地地址*/
	g_tModH.TxBuf[g_tModH.TxCount++] = fun_code;		/* 功能码 等离子开或者关闭 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = len;	/* 数据长度*/
	g_tModH.TxBuf[g_tModH.TxCount++] = data;		/* 数据 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck02H = 0;		/* 清接收标志 */
	//g_tModH.RegNum = _num;		/* 寄存器个数 */
	//g_tModH.Reg02H = _reg;		/* 保存02H指令中的寄存器地址，方便对应答数据进行分类 */	



}

/*********************************************************************************************************
*	函 数 名: MODH_WriteParam_10H
*	功能说明: 单个参数. 通过发送10H指令实现，发送之后，等待从机应答。循环NUM次写命令
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************/
void RS485_Host_Communication_Handler(void)
{

    static uint8_t rs485_run_flag,dry_flag=0xff,plasma_flag=0xff; 
	static uint8_t  dry_off_flag=0xff,plasma_off_flag =0xff;
	 switch(run_t.rs485_Command_tag){


	 case POWER_ON:
	 	 
	    MODH_WriteParam_Power_01H(0x00,0x01,0x01); //all local extension machine power on  
	    
	    rs485_run_flag =1;

	    run_t.rs485_Command_tag =0xff;

	 break;
     case POWER_OFF: //0
        
	 	MODH_WriteParam_Power_01H(0x00,0x01,0);
    
		rs485_run_flag =0;
	   
	   run_t.rs485_Command_tag =0xff; 

	 break;


	}
     
    if(rs485_run_flag ==1){

	  // bsp_Idle();
   
	   if(run_t.gDry ==1 && (dry_flag !=run_t.rs485_send_dry)){

	       dry_flag = run_t.rs485_send_dry;

		   MODH_WriteParam_PTC_02H(0x00,0x01,0x01);
		   

		}
		else if(run_t.gDry ==0 && (dry_off_flag !=run_t.rs485_send_dry)){
			dry_off_flag =run_t.rs485_send_dry;
		   MODH_WriteParam_PTC_02H(0x00,0x01,0x0);
		

		}


		if(run_t.gPlasma ==1 && (plasma_flag !=run_t.rs485_send_plasma)){
			  plasma_flag = run_t.rs485_send_plasma;
          
		     MODH_WriteParam_Plasma_03H(0x00,0x01,1);


		}
		else if(run_t.gPlasma ==0 && (plasma_off_flag !=run_t.rs485_send_plasma)){
			
		   plasma_off_flag = run_t.rs485_send_plasma;

		   MODH_WriteParam_Plasma_03H(0x00,0x01,0x00);


		}

   }

}


