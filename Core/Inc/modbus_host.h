#ifndef __MODBUS_HOST_H_
#define __MODBUS_HOST_H_
#include "main.h"


#define MasterAddr		0x01		/* 主机地址 */
#define SlaveAddr_1		0x02		/* 1号分机,address */
#define SlaveAddr_2		0x03		/* 2号分机 ,address*/
#define SlaveAddr_3		0x04		/* 3号分机 ,address*/
#define SlaveAddr_4		0x05		/* 4号分机 ,address*/

/*Function Codes*/
#define FUN_POWER  		    0x01
#define FUN_OPEN            0x02
#define FUN_PLASMA     		0x03
#define FUN_ULTRASONIC 		0x04
#define FUN_FAN				0x05

/*Data Codes*/
#define DATA_OPEN           0x01
#define DATA_CLOSE          0x00

/* 01H 读强制单线圈 */
/* 05H 写强制单线圈 */
#define REG_D01		0x0101
#define REG_D02		0x0102
#define REG_D03		0x0103
#define REG_D04		0x0104
#define REG_DXX 	REG_D04

/* 02H 读取输入状态 */
#define REG_T01		0x0201
#define REG_T02		0x0202
#define REG_T03		0x0203
#define REG_TXX		REG_T03

/* 03H 读保持寄存器 */
/* 06H 写保持寄存器 */
/* 10H 写多个保存寄存器 */
#define REG_P01		0x0301		
#define REG_P02		0x0302	

/* 04H 读取输入寄存器(模拟信号) */
#define REG_A01		0x0401
#define REG_AXX		REG_A01

/* RTU 应答代码 */
#define RSP_OK				0		/* 成功 */
#define RSP_ERR_CMD			0x01	/* 不支持的功能码 */
#define RSP_ERR_REG_ADDR	0x02	/* 寄存器地址错误 */
#define RSP_ERR_VALUE		0x03	/* 数据值域错误 */
#define RSP_ERR_WRITE		0x04	/* 写入失败 */

#define H_RX_BUF_SIZE		20//64
#define H_TX_BUF_SIZE      	32//128

#if UART2_FIFO_EN == 1
	#define UART2_BAUD			9600
	#define UART3_TX_BUF_SIZE	1*1024
	#define UART3_RX_BUF_SIZE	1*1024
#endif

typedef enum {

  mod_power =0x01,
  mod_ptc,
  mod_plasma,
  mod_ulrasonic,
  mod_fan
   
}_mod_fun;

typedef enum{

  rx_rs485_data_fail,
  rx_rs485_data_success

}Rx_rs485_data;

typedef enum _mod_fun mod_fun_codes;


typedef struct
{
	uint8_t RxBuf[H_RX_BUF_SIZE];
	uint8_t RxCount;
	uint8_t RxStatus;
	uint8_t RxNewFlag;
	uint8_t rs485_RxInputBuf[1];
	uint8_t Rx_rs485_data_flag;

	uint8_t RspCode;

	uint8_t TxBuf[H_TX_BUF_SIZE];
	uint8_t TxCount;
	
	uint8_t mod_power;//Reg01H;		/* 保存主机发送的寄存器首地址 */
	uint8_t mod_ptc;//Reg02H;
	uint8_t mod_plasma;//Reg03H;		
	uint8_t mod_fan;//Reg04H;

	uint8_t RegNum;			/* 寄存器个数 */

	uint8_t fAck01H;		/* 应答命令标志 0 表示执行失败 1表示执行成功 */
	uint8_t fAck02H;
	uint8_t fAck03H;
	uint8_t fAck04H;
	uint8_t fAck05H;		
	uint8_t fAck06H;		
	uint8_t fAck10H;
	
}MODH_T;

extern MODH_T g_tModH;

typedef struct
{
	/* 03H 06H 读写保持寄存器 */
	uint16_t P01;
	uint16_t P02;
	
	/* 02H 读写离散输入寄存器 */
	uint16_t T01;
	uint16_t T02;
	uint16_t T03;
	
	/* 04H 读取模拟量寄存器 */
	uint16_t A01;
	
	/* 01H 05H 读写单个强制线圈 */
	uint16_t D01;
	uint16_t D02;
	uint16_t D03;
	uint16_t D04;
	
}VAR_T;



void MODH_Poll(void);
uint8_t MODH_ReadParam_Power_01H(uint8_t addr,uint8_t _len,uint8_t _reg);
uint8_t MODH_ReadParam_PTC_02H(uint8_t addr,uint8_t _len,uint8_t _reg);
uint8_t MODH_ReadParam_Plasma_03H(uint8_t addr,uint8_t _len,uint8_t _reg);
uint8_t MODH_ReadParam_Ultrasonic_04H(uint8_t addr,uint8_t _len,uint8_t _reg);
uint8_t MODH_WriteParam_05H(uint16_t _reg, uint16_t _value);
uint8_t MODH_WriteParam_06H(uint16_t _reg, uint16_t _value);
uint8_t MODH_WriteParam_10H(uint16_t _reg, uint8_t _num, uint8_t *_buf);
void RS485_Host_Communication_Handler(void);










#endif 

