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
#define BSP_TIMEOUT		1		/* 接收命令超时时间, 单位ms */
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

MODH_T g_tModH ;
Protocol_t g_tPro;

uint8_t crc16_check_flag;
uint8_t rs485_rx_local[10];

static void Rx485_Receive_Slave_Id(uint16_t id);

static uint8_t findMaxPos(uint16_t *ptarr,uint8_t n);
static void Selection_Sort(uint16_t *ptarr,uint8_t len);
static void MODH_Read_Address_Info(void);






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
/*********************************************************************************************************
*
*	函 数 名: void MODH_Broadcast_Mode(uint8_t fun_code,uint8_t _data)
*	功能说明: 主机广播模式
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*
*********************************************************************************************************/
void MODH_Broadcast_Mode(uint8_t fun_code,uint8_t _data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = HEAD_FLAG;  
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x55;
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x55;
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x00;		/* 从站地址 发送地址 ,高字节数据，大端发送*/	
	g_tModH.TxBuf[g_tModH.TxCount++] = MasterAddr;	/* 从站地址 发送地址 ,低字节数据，大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = fun_code;		/* 功能码 开机或者关机 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x01;	/* 数据长度*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _data;		/* 数据 */	


	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck01H = 0;		/* 清接收标志 */

}

/*********************************************************************************************************
*	函 数 名: MODH_Send00H
*	功能说明: 发送00H指令，查询所有从机线圈寄存器 功能码 0x01
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************/

void MODH_Send00H_Power_OnOff(uint16_t _addr, uint8_t _data_len,uint8_t _data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = HEAD_FLAG;  
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr >>8;		/* 从站地址 发送地址 ,高字节数据，大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* 从站地址 发送地址 ,低字节数据，大端发送*/
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
void MODH_Send00H_Ptc_OnOff(uint16_t _addr,uint8_t _data_len,uint8_t _data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = HEAD_FLAG;  
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr >>8;		/* 从站地址 发送地址 ,高字节数据，大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* 从站地址 发送地址 ,低字节数据，大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = MasterAddr;  /* 本地地址*/
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x02;		/* 功能码 PTC加开或者关闭 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _data_len;	/* 数据长度*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _data;		/* 数据 */	
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck02H = 0;		/* 清接收标志 */
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
void MODH_Send00H_Plasma_OnOff(uint16_t _addr,uint8_t _data_len,uint8_t _data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = HEAD_FLAG;  
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr >>8;		/* 从站地址 发送地址 ,高字节数据，大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* 从站地址 发送地址 ,低字节数据，大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = MasterAddr;  /* 本地地址*/
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x03;		/* 功能码 等离子开或者关闭 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _data_len;	/* 数据长度*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _data;		/* 数据 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck03H = 0;		/* 清接收标志 */
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
void MODH_Send00H_Ultrasonic_OnOff(uint16_t _addr,uint8_t _data_len,uint8_t _data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = HEAD_FLAG;  
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr >>8;		/* 从站地址 发送地址 ,高字节数据，大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* 从站地址 发送地址 ,低字节数据，大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = MasterAddr;  /* 本地地址*/
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x04;		/* 功能码 超声波开或者关闭 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _data_len;	/* 数据长度*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _data;		/* 数据 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck04H = 0;		/* 清接收标志 */
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
void MODH_Send00H_Fan_OnOff(uint16_t _addr,uint8_t _data_len,uint8_t _data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = HEAD_FLAG;  
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr >>8;		/* 从站地址 发送地址 ,高字节数据，大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* 从站地址 发送地址 ,低字节数据，大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = MasterAddr;  /* 本地地址*/
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x05;		/* 功能码 风扇开或者关闭 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _data_len;	/* 数据长度*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _data;		/* 数据 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck05H = 0;		/* 清接收标志 */
	//g_tModH.RegNum = _num;		/* 寄存器个数 */
	//g_tModH.Reg04H = _reg;		/* 保存04H指令中的寄存器地址，方便对应答数据进行分类 */	
}
/*
*********************************************************************************************************
*	函 数 名: MODH_SendB0H
*	功能说明: 发送B4H指令，设置温度值
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_SendB0H_SetTemperature_Value(uint16_t _addr,uint8_t _data_len,uint8_t _data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = HEAD_FLAG;  
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr >>8;		/* 从站地址 发送地址 ,高字节数据，大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* 从站地址 发送地址 ,低字节数据，大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = MasterAddr;  /* 本地地址*/
	g_tModH.TxBuf[g_tModH.TxCount++] = 0xB0;		/* 功能码 风扇开或者关闭 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _data_len;	/* 数据长度*/
	g_tModH.TxBuf[g_tModH.TxCount++] = _data;		/* 数据 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAckb0H = 0;		/* 清接收标志 */
	//g_tModH.RegNum = _num;		/* 寄存器个数 */
	//g_tModH.Reg04H = _reg;		/* 保存04H指令中的寄存器地址，方便对应答数据进行分类 */	
}
/*

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
	uint16_t crc1 ,addr;
	if(g_tModH.Rx_rs485_data_flag == rx_rs485_data_success){
		/* 计算CRC校验和，这里是将接收到的数据包含CRC16值一起做CRC16，结果是0，表示正确接收 */
	     g_tPro.pro_addr = BEBufToUint16(rs485_rx_local);//rs485_rx_local[1];

	     g_tPro.pro_local_addr = BEBufToUint16_SlaveAddress(rs485_rx_local);
	
		crc1 = CRC16_Modbus(g_tModH.RxBuf,g_tModH.RxCount);
		if (crc1 != 0)
		{
			g_tModH.RxCount = 0;	/* 必须清零计数器，方便下次帧同步 */
			g_tModH.Rx_rs485_data_flag=0;
			//if(g_tModH.RxBuf[0]==0)run_t.broadcast_response_signal = FAIL_BROADCAST;
			addr = BEBufToUint16_SlaveAddress(g_tModH.RxBuf);
			Answerback_RS485_Signal(addr,0xff,g_tModH.RxBuf[3],0x01);
		}
		else{
        	crc16_check_flag = 1;
			
		   // memcpy(rs485_rx_local,g_tModH.RxBuf,7);
				/* 必须清零计数器，方便下次帧同步 */
		
		}
    }
	if(crc16_check_flag==1){
		/* 分析应用层协议 */
		MODH_AnalyzeApp();
		
		crc16_check_flag=0;
		g_tModH.RxCount = 0;
		g_tModH.Rx_rs485_data_flag=0;
	 
	
   	}

	
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
	
    MODH_Read_Address_Info();
	   
}
/*******************************************************************************************************
*
*	函 数 名: MODH_AnalyzeApp
*	功能说明: 分析应用层协议。处理应答。
*	形    参: 无
*	返 回 值: 无
*
*******************************************************************************************************/
static void MODH_Read_Address_Info(void)
{
   
 

   	// g_tPro.pro_addr = BEBufToUint16(rs485_rx_local);//rs485_rx_local[1];

	// g_tPro.pro_local_addr = BEBufToUint16_SlaveAddress(rs485_rx_local);
	 
     g_tPro.pro_fun_code = rs485_rx_local[5];

	 g_tPro.pro_data_len = rs485_rx_local[6];

	g_tPro.pro_data = rs485_rx_local[7];
	

  if(g_tPro.pro_addr == 0X0001){ //host of address 

      g_tModH.rx485_rx_data_flag = 1;
	  Answerback_RS485_Signal(g_tPro.pro_local_addr,g_tPro.pro_fun_code,g_tPro.pro_data_len,g_tPro.pro_data);
      g_tPro.pro_addr=0xff;
  }

  if(g_tModH.rx485_rx_data_flag == 1){

    switch(g_tPro.pro_fun_code){
  
          case slave_fault :

	            switch(g_tPro.pro_data){

				   case ptc_trouble:

				    g_tModH.rs485_ext_fault_ptc =1 ;  
				    g_tModH.gTimer_fault_ptc_times = 12;//at once run this action

				   break;

				   case fan_trouble:

				    g_tModH.rs485_ext_fault_fan =1;
					g_tModH.gTimer_fault_fan_times = 12; //at once run this action

				   break;



				}
        	
		   g_tModH.rx485_rx_data_flag =0xff;
		  break;

		  case slave_address: //

		   Rx485_Receive_Slave_Id(g_tPro.pro_local_addr);
            
          g_tModH.rx485_rx_data_flag =0xff;
		  break;

		  case 2:
		

		  break;

		  case 3:


		  break;

    	}
	 }

}
  
/**********************************************************************************************************
*	函 数 名: MODH_Read_01H
*	功能说明: 分析01H指令的应答数据，读取线圈状态，bit访问
*             [发送的地址]+[本机地址]+[功能码]+[数据长度]+[数据]+[CRC16低]+[CRC16高]
*	形    参: 无
*	返 回 值: 无
**********************************************************************************************************/

/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_01H
*	功能说明: 单个参数. 通过发送01H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_Power_01H(uint16_t add,uint8_t _len,uint8_t _reg)
{
	//int32_t time1,time2;
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		/* 发送命令 */
		MODH_Send00H_Power_OnOff(add,_len,_reg);
		g_tModH.gTimer_rs485_rx_times=0;

	

		//time1 = bsp_GetRunTime();	/* ¼ÇÂ¼ÃüÁî·¢ËÍµÄÊ±¿Ì */
		
		while (1)				/* µÈ´ýÓ¦´ð,³¬Ê±»ò½ÓÊÕµ½Ó¦´ðÔòbreak  */
		{
			bsp_Idle();

			if (g_tModH.gTimer_rs485_rx_times >BSP_TIMEOUT)		
			{
				break;		/* timer timing is over 1s */
			}
			
			if (g_tModH.fAck01H > 0)
			{
				break;		/* ½ÓÊÕµ½Ó¦´ð */
			}
		}
		
		if (g_tModH.fAck01H > 0)
		{
			break;			/* Ñ­»·NUM´Î£¬Èç¹û½ÓÊÕµ½ÃüÁîÔòbreakÑ­»· */
		}	
		
		
	}

	if (g_tModH.fAck01H == 0)
	{
		return 0;
	}
	else 
	{
		return 1;	/* 01H ¶Á³É¹¦ */
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
uint8_t MODH_WriteParam_PTC_02H(uint16_t add,uint8_t _num,uint8_t _reg)
{
	
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		
		MODH_Send00H_Ptc_OnOff(add,_num,_reg);
		g_tModH.gTimer_rs485_rx_times=0;
		
		while (1)
		{
			bsp_Idle();

			if (g_tModH.gTimer_rs485_rx_times > BSP_TIMEOUT)		
			{
				break;		/* Í¨ÐÅ³¬Ê±ÁË */
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
uint8_t MODH_WriteParam_Plasma_03H(uint16_t add,uint8_t len,uint8_t _reg)
{
	int32_t time1;
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		//MODH_Send03H (SlaveAddr_1, _reg, _num);
		MODH_Send00H_Plasma_OnOff(add,len,_reg);
		// g_tModH.rx485_send_fun_code = plasma_order;
		 g_tModH.gTimer_rs485_rx_times=0;
	

		while (1)
		{
			bsp_Idle();

			if (g_tModH.gTimer_rs485_rx_times >BSP_TIMEOUT)		
			{
				break;		/* Í¨ÐÅ³¬Ê±ÁË */
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
		return 0;
	}
	else 
	{
		return 1;	/* 02H 读成功 */
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
uint8_t MODH_WriteParam_Ultrasonic_04H(uint16_t add,uint8_t _num,uint8_t _reg)
{
	int32_t time1;
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		
		MODH_Send00H_Ultrasonic_OnOff(add,_num,_reg);
        // g_tModH.rx485_send_fun_code = ultrasonic_order;
		  g_tModH.gTimer_rs485_rx_times=0;
	
		while (1)
		{
			bsp_Idle();

			if (g_tModH.gTimer_rs485_rx_times >BSP_TIMEOUT)		
			{
				break;		/* Í¨ÐÅ³¬Ê±ÁË */
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
		return 0;
	}
	else 
	{
		return 1;	/* 02H 读成功 */
	}
	
	
}
/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_05H
*	功能说明: 单个参数. 通过发送05H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_Fan_05H(uint16_t add,uint8_t _num,uint8_t _reg)
{
	
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		//MODH_Send04H (SlaveAddr_1, _reg, _num);
		MODH_Send00H_Fan_OnOff(add,_num,_reg);
     
		g_tModH.gTimer_rs485_rx_times=0;
		
		while (1)
		{
			bsp_Idle();

			if (g_tModH.gTimer_rs485_rx_times >BSP_TIMEOUT)		
			{
				break;		/* Í¨ÐÅ³¬Ê±ÁË */
			}
			
			if (g_tModH.fAck05H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck05H > 0)
		{
			break;
		}
		
	}
	
	if (g_tModH.fAck05H == 0)
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
*	函 数 名: MODH_ReadParam_B0H
*	功能说明: 单个参数. 通过发送05H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_SetTempValue_B0H(uint16_t add,uint8_t _len,uint8_t _data)
{
	
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		//MODH_Send04H (SlaveAddr_1, _reg, _num);
		MODH_SendB0H_SetTemperature_Value(add,_len,_data);
  
		g_tModH.gTimer_rs485_rx_times=0;
		
	
		while (1)
		{
			bsp_Idle();

			if (g_tModH.gTimer_rs485_rx_times >BSP_TIMEOUT)		
			{
				break;		/* Í¨ÐÅ³¬Ê±ÁË */
			}
			
			if (g_tModH.fAckb0H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAckb0H > 0)
		{
			break;
		}
		
	}
	
	if (g_tModH.fAckb0H == 0)
	{
		return 0;
	}
	else 
	{
		return 1;	/* b0H 读成功 */
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
void Answerback_RS485_Signal(uint16_t addr ,uint8_t fun_code,uint8_t len,uint8_t data)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++]= HEAD_FLAG;
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x00;
	g_tModH.TxBuf[g_tModH.TxCount++] = MasterAddr;		/* 主站地址 发送地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = addr >> 8;  	/* 本地地址 数据高字节 数据大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = addr;  		/* 本地地址 数据低字节 数据大端发送*/
	g_tModH.TxBuf[g_tModH.TxCount++] = fun_code;		/* 功能码   ，开机，PTC，杀菌，驱鼠 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = len;	/* 数据长度*/
	g_tModH.TxBuf[g_tModH.TxCount++] = data;		/* 数据 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	/* 清接收标志 */
	//g_tModH.RegNum = _num;		/* 寄存器个数 */
	//g_tModH.Reg02H = _reg;		/* 保存02H指令中的寄存器地址，方便对应答数据进行分类 */	



}

/*********************************************************************************************************
*	函 数 名: MODH_WriteParam_10H
*	功能说明: 单个参数. 通过发送10H指令实现，发送之后，等待从机应答。循环NUM次写命令
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************/
void RS485_Host_Send_Communication_Handler(void)
{
    if(g_tModH.gTimer_rs485_run_times > 0){
	     g_tModH.gTimer_rs485_run_times=0;

	   
	 switch(g_tModH.rs485_Command_tag){


	 case POWER_ON:
	 	 
	    //MODH_WriteParam_Power_01H(0x00,0x01,0x01); //all local extension machine power on 
		 MODH_Broadcast_Mode(power_cmd,POWER_ON);	    

	    g_tModH.rs485_Command_tag =0xff;

	 break;
     case POWER_OFF: //0
        
	 	//MODH_WriteParam_Power_01H(0x00,0x01,0);
	 	 MODH_Broadcast_Mode(power_cmd,POWER_OFF);	   
    
	
	   
	   g_tModH.rs485_Command_tag =0xff; 

	 break;

	 case DRY_ON:
	 	//MODH_WriteParam_PTC_02H(0x00,0x01,0x01);
	 	 MODH_Broadcast_Mode(ptc_cmd,open);	   

	 	 g_tModH.rs485_Command_tag =0xff; 

	 break;

	 case DRY_OFF:
	 	//MODH_WriteParam_PTC_02H(0x00,0x01,0x0);
		 MODH_Broadcast_Mode(ptc_cmd,off);	  
	    g_tModH.rs485_Command_tag =0xff; 

	 break;

	 case PLASMA_ON:
	 	 //MODH_WriteParam_Plasma_03H(0x00,0x01,1);
          MODH_Broadcast_Mode(plasma_cmd,open);	  
	     g_tModH.rs485_Command_tag =0xff; 

	 break;

	 case PLASMA_OFF:
	 	 ///MODH_WriteParam_Plasma_03H(0x00,0x01,0x00);
        MODH_Broadcast_Mode(plasma_cmd,off);	  
	  g_tModH.rs485_Command_tag =0xff; 

	 break;

	 case DRIVE_RAT_ON:
	 	 ///MODH_WriteParam_Ultrasonic_04H(0x00,0x01,0x01);
	 	  MODH_Broadcast_Mode(ultrasonic_cmd,open);	  
		  g_tModH.rs485_Command_tag =0xff; 

	 break;

	 case DRIVE_RAT_OFF:
	 	 //MODH_WriteParam_Ultrasonic_04H(0x00,0x01,0x00);
	 	 MODH_Broadcast_Mode(ultrasonic_cmd,off);	  
		  g_tModH.rs485_Command_tag =0xff; 

	 break;

	 case RS485_TEMP:
		///MODH_WriteParam_SetTempValue_B0H(0x00,0x01,g_tModH.set_temperature_value);
		 MODH_Broadcast_Mode(temp_cmd,g_tModH.rs485_set_temperature_value);	  
		 g_tModH.rs485_Command_tag =0xff; 

	 break;




	}

    }
     
  }

/*****************************************************************************
	*
	*Function Name: static vid Rx485_Receive_Slave_Id(void)
	*Function:
	*Input Ref: 
	*Return Ref:
	*
******************************************************************************/
static void Rx485_Receive_Slave_Id(uint16_t id)
{

       static uint8_t slave_id_one,slave_id_two,slave_id_three,slave_id_four;

	   if(g_tModH.slave_Id[3]!=0){
           Selection_Sort(g_tModH.slave_Id,4);

	   }
	   else{

	   if(g_tModH.slave_Id[0]==0)g_tModH.slave_Id[0]=id;
	   else{

		   if(g_tModH.slave_Id[0]!=0){

	          if(g_tModH.slave_Id[0]==id){
			  	g_tModH.rx485_rx_data_flag =0xff;
			  	return;
	          }
			  else if(g_tModH.slave_Id[1] ==0){

				g_tModH.slave_Id[1]=id;
				return;

			  }
		   	 }
	   	}

		if(g_tModH.slave_Id[1]!=0){

		    if(g_tModH.slave_Id[1]==id){
				g_tModH.rx485_rx_data_flag =0xff;
		  	      return;
		    }
		    else if(g_tModH.slave_Id[2] ==0){

				g_tModH.slave_Id[2]=id;
				return;

		   }
		}
		
	    if(g_tModH.slave_Id[2]!=0){
			 	
		  	if(g_tModH.slave_Id[2]==id){
				g_tModH.rx485_rx_data_flag =0xff;
		  	      return;
		    }
		  	else if(g_tModH.slave_Id[3] ==0){

				g_tModH.slave_Id[3]=id;
				return;

		   }
		}
		
	   if(g_tModH.slave_Id[3]!=0){
			  	
					
			  	if(g_tModH.slave_Id[3]==id){
					 g_tModH.rx485_rx_data_flag =0xff;
			  	      return;
			    }
			    
			  }
			  
	   	}

 }

/*****************************************************************************
	*
	*Function Name: void Select_Sort(int *pt,int n)
	*Function:
	*Input Ref: *pt ->input array ,  n-->array of length
	*
	*
******************************************************************************/
static uint8_t findMaxPos(uint16_t *ptarr,uint8_t n)
{
     uint8_t i;
	 uint8_t pos;
	 uint16_t max;
	 max = ptarr[0];
	 pos = 0;

	 for(i=0; i<n;i++){

        if(ptarr[i] > max){
           max = ptarr[i];
		   pos = i;

		}


	 }

	return pos;

}


static void Selection_Sort(uint16_t *ptarr,uint8_t len)
{
   uint8_t  pos;
   uint16_t temp;
  // len = sizeof(ptarr)/sizeof(ptarr[0]);

   while(len > 1){

       pos = findMaxPos(ptarr,len);
	   temp = ptarr[pos];
       ptarr[pos] = ptarr[len-1];
	   ptarr[len-1] = temp;
	   len--;
   }
  
}

