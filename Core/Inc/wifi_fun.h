#ifndef __WIFI_FUN_
#define __WIFI_FUN_
#include "main.h"
#include "cmd_link.h"


#define WIFI_AUTO_SMART_CONFIG()       				HAL_GPIO_WritePin(WIFI_CONFIG_GPIO_Port,WIFI_CONFIG_Pin,GPIO_PIN_RESET)
#define WIFI_AUTO_EXIT_SMART_CONFIG()				HAL_GPIO_WritePin(WIFI_CONFIG_GPIO_Port,WIFI_CONFIG_Pin,GPIO_PIN_SET)


typedef enum DISPLAY_STATE_T{
   
    WIFI_POWER_ON = 0x80,
    WIFI_POWER_OFF=0X81,
    WIFI_MODE_1=0X08,   //state ->normal works
    WIFI_MODE_2=0X18,   //state->sleeping works
    WIFI_KILL_ON=0x04,  //Anion(plasma)
    WIFI_KILL_OFF=0x14,
    WIFI_PTC_ON = 0x02, 
    WIFI_PTC_OFF = 0x12,
    WIFI_ULTRASONIC_ON = 0x01,       //ultrasonic
    WIFI_ULTRASONIC_OFF = 0x11,
    WIFI_WIND_SPEED= 0x90,
    WIFI_TEMPERATURE = 0xA0,
    WIFI_POWER_ON_NORMAL= 0xB0


}display_state_t;

typedef enum _CLOUD_STATE{
   OPEN_OFF_ITEM=0x01,
   OPEN_ON_ITEM,
   PTC_OFF_ITEM,
   PTC_ON_ITEM,
   STERILIZATION_OFF_ITEM,
   STERILIZATION_ON_ITEM,
   ULTRASONIC_OFF_ITEM,
   ULTRASONIC_ON_ITEM,
   STATE_ON_ITEM,
   STATE_OFF_ITEM,
   TEMPERATURE_ITEM,
   FAN_ITEM
}cloud_state;



typedef enum _subSignal{ /* enumeration for CParser signals */
   subscrible_data,
   OPEN_SIG, 
   STATE_SIG, 
   PTC_SIG,
   SONIC_SIG,
   ANION_SIG,
   TEMP_SIG,
   FIND_SIG,
   HUM_SIG,
   NOWTEMP_SIG
}subSignal;



typedef enum{
  
   wifi_AI=0x08 ,
   wifi_notAI=0x18,
   wifi_kill=0x04,
   wifi_notkill=0x14,
   wifi_heat= 0x02,
   wifi_notheat=0x12


}wifi_mode_t;


typedef enum{

   WIFI_DONOT_PASS,
   WIFI_PASS

}wifi_pass_state;


typedef enum {

	wifi_model_device_report =0x01,
	wifi_model_device_issue =0x02,
    wifi_model_operation =0xFE,
    wifi_model_frame_error =0xFF
  
}wifi_frame_type;

typedef enum{

	wifi_model_state_query = 0x01,
	wifi_model_smartconfig_order = 0x3,
	wifi_model_prodkey_order = 21
}wifi_enum_order;



typedef enum{
	wifi_model_sta = 0x01,
	wifi_model_hekr_config,
	wifi_model_compatible = 0x04

}wifi_model_enum_t;

typedef enum{
	wifi_model_normal_link = 0x01,
    wifi_model_link_fail = 0x02,
    wifi_model_linking = 0x03,
    wifi_model_password_error = 0x04,
	wifi_model_no_lookfor_router = 0x05,
    wifi_model_link_times_overflow = 0x06,
    wifi_model_no_link_router = 0x07,

	wifi_model_exit
   

}wifi_model_wifi_state_t;





typedef struct _WIFI_FUN{
	
    uint8_t wifi_dispose_data[USART_WIFI_NUMBERS];
	
    uint8_t usart_wifi_frame_len;
    uint8_t usart_wifi_frame_type;
    uint8_t usart_wifi_sequence;
    uint8_t usart_wifi_order;
    uint8_t usart_wifi_model;
    uint8_t usart_wifi_state;
    uint8_t usart_wifi_cloud_state;
    uint8_t usart_wifi_signal_state;
	uint8_t usart_wifi_sum_codes;
    uint8_t usart_wifi_pass_state;
	uint8_t wifi_receive_data_codes_sum_flag;
    uint8_t wifi_receive_data_error;
	

	uint8_t wifi_receive_data_state;
    uint8_t BJ_time_hours;
	uint8_t BJ_time_minutes;
    uint8_t BJ_time_seconds;
 
	uint8_t wifi_link_JPai_cloud;
	uint8_t usart_wifi_seconds_value;
	uint8_t usart_wifi_fan_speed_value;
	uint8_t publish_send_state_data;
	uint8_t wifi_has_been_link_cloud;
	uint8_t gTimer_wifi_send_cloud_success_times;
	uint8_t wifi_open_power_on_flag;


    uint8_t real_hours;
	uint8_t real_minutes;
	uint8_t real_seconds;
	uint8_t restart_link_tencent_cloud ;

    uint8_t gTimer_1s;
	uint8_t gTimer_5s;
    uint8_t gTimer_get_beijing_time;
	uint8_t gTimer_beijing_time;

	
		
	
}WIFI_FUN;

extern WIFI_FUN   wifi_t;

extern void (*PowerOn)(void); //函数指针
extern void (*PowerOff)(void);
extern void (*Ai_Fun)(uint8_t sig);
extern void (*SetTimes)(void);
extern void (*SetTemperature)(void);
extern uint8_t (*wifi_run_state_func)(void);





void wifiDisplayTemperature_Humidity(void);


void wifiUpdate_SetTimeValue(uint8_t tv);
void wifiUpdate_SetTemperatureValue(uint8_t temp);

void RunWifi_Command_Handler(uint8_t cmd);
void GetNTP_Times(void);


void Wifi_Model_State_Handler(uint8_t (*wifi_state_fun)(void));

void Read_USART2_Wifi_Data(uint8_t sel,uint8_t len,uint8_t cmd);


#endif 

