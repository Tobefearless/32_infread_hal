#ifndef __CONFIG_H
#define __CONFIG_H
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include <stdio.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "FreeRTOS.h" 
#include "task.h"
#include "flash.h"
//#include "wifi.h"
#include "crc.h"
#include <time.h>

#pragma pack(1)

//#define DEV_ID "SQLS210300002"

/****************************定义各数据包头和尾**************************/
typedef enum{
	TABLE_HEAD = 0XA0, 
	TABLE_TAIL = 0XA1,
	TO_APP_HEAD = 0xA2,
	TO_APP_TAIL = 0xA3,
	_485_CMD_HEAD = 0xA4,
	_485_CMD_TAIL = 0xA5,
	WIFI_SSID_HEAD = 0xA6,
	WIFI_SSID_TAIL = 0xA7,
	TIME_TAMP_HEAD = 0xA8,
	TIME_TAMP_TAIL = 0xA9,
	CHANGE_ID_HEAD = 0XAA,
  CHANGE_ID_TAIL = 0XAB,
  GET_DEV_STATUS_HEAD = 0XB1,
	GET_DEV_STATUS_TAIL = 0XB2,
	GET_NET_TIME_HEAD = 0XB3,
	GET_NET_TIME_TAIL = 0XB4,
	GET_AIR_CODE_HEAD   =   0xB5,
	GET_AIR_CODE_TAIL   =   0xB6,
}HEAD_TAIL_CMD;

typedef struct{
bool enable;
bool wif_conect_status;
char wifi_con[50];
char mqtt_con[60];
char mqtt_pub[100];
char mqtt_sub[100];
char name[25];
char pasw[25];
char id[200];
}WIFI_CFG_Typedef;

/*****************************一 下发到单片机的表 ***************************/
  /*(一).各参数表具体类型*/
    /*1.换气参数表(大端数据格式)(8条)*/

		typedef struct{
			time_t  time_sta;
			uint8_t min_len;
		}SET_BREATH_TypeDef;
	   
	/*2.消毒参数表(大端数据格式)(4条)*/
		typedef struct{
			time_t  time_sta;
			uint8_t min_len;
		}SET_STERILIZE_TypeDef;
	/*3.光照参数表(大端数据格式))(3条)*/
		typedef struct{
			time_t  time_sta;
			time_t  time_end;
		}SET_LIGHT_TypeDef;
	/*4.温湿度设定表(大端数据格式)))(1条)//保留一位小数*/
		typedef struct{
			uint16_t temp_center;
			uint16_t temp_diff;
			uint16_t humi_center;
			uint16_t humi_diff;
		}SET_SENSOR_Type;
	/*5.感光参数表(大端数据格式)(3条)*/
		typedef struct{
			time_t  time_sta;
			time_t  time_end;
		}SET_SENSITIVE_TypeDef;
	/*6.传感器校准值(大端数据格式)(4条)//保留一位小数*/
		typedef struct{
			int16_t temp_adjust;
			int16_t humi_adjust;	
		}SET_SENSOR_DIFF_TypeDef;
/******************************	(二).统一各参数表和设置 **************************/ 
     typedef struct{
		HEAD_TAIL_CMD head;
		char id[20];
		SET_BREATH_TypeDef set_breath[8];
		SET_STERILIZE_TypeDef set_sterilize[4];
		SET_LIGHT_TypeDef set_light[3];
		SET_SENSOR_Type  set_sensor[1];
		SET_SENSITIVE_TypeDef set_sensitive[3];
		SET_SENSOR_DIFF_TypeDef set_senser_diff[6];
		uint8_t set_wind_temp_diff; 
		HEAD_TAIL_CMD tail;
		uint16_t crc;
	 }UPDATE_ALL_TABLE_TYPEDEF;
	 
	 /***pc端和网络端和触摸屏任意更新一个都存到flah其他全部更新***/
	 
	 
	 
/*************************************** 二  控制器发送温湿度、控制状态等信息到pc端:****************************************/
  /*(一)、控制器具体各参数*/
   /* 1.开关状态结构体*/
        typedef struct{
			uint8_t heat:1;
			uint8_t humi:1;
			uint8_t dry:1;
			uint8_t breath:1;
			uint8_t wind:1;
			uint8_t light:1;//光照状态
			uint8_t sterilize:1;//消毒状态
			uint8_t sensitive:1;//感光状态
		}RELAY_Typedef;
	/*2.空调状态枚举	*/
		typedef enum {
			AIR_CLOSE = 0,
			AIR_HEAT = 1,
			AIR_COLD = 2,
		}AIR_STATUS;
		
	/*3.单个传感器数据*/
			 typedef enum{
				SENSER_OK = 0,
				SENSER_DIS_CON = 1,
				SENSER_FAIL = 2,
			}SENSER_STATUS;
			 
	    typedef struct{
		  uint16_t single_temp;
		  uint16_t signle_humi;
			SENSER_STATUS senser_status;
		}SINGLE_SENSER_Typedef;
 /*(二)、控制器最终发送各参数*/		
		typedef union {
			RELAY_Typedef relay;
			uint8_t relay_vale;
		}RELAY;
		typedef struct{
		    HEAD_TAIL_CMD head;      
			char id[20];
			uint16_t temp;
			uint16_t humi;
      SET_SENSOR_Type now_sensor;
			SET_LIGHT_TypeDef next_light;
			SET_BREATH_TypeDef next_breath;
			SET_STERILIZE_TypeDef next_sterilize;
			SET_SENSITIVE_TypeDef next_sensitive;//感光
			SINGLE_SENSER_Typedef signle_senser[6];
			AIR_STATUS air_status;
			RELAY relay_status;
			HEAD_TAIL_CMD tail;
			uint16_t crc;
		}TO_APP_Typedef;

		
	
/***********************************三  485-控制器指令数据格式: ************************************/
	typedef enum{
				GET_WIFI_SCAN = 0,//发送wifi扫描指令
				WIFI_CON_STA,//连接wifi
				WIFI_CON_STOP,//断开wifi
				WIFI_CON_OK,//告诉pc端wifi连接成功
				WIFI_CON_FAIL,//连接失败
				_4G_CON_STA,//连接4G
				_4G_CON_OK,//连接4G OK
				_4G_CON_FAIL,//连接4G 失败
				AIR_HEAT_LEARN_STA,//开始空调制热学习
				AIR_HEAT_LEARN_END,//结束空调制热学习
				AIR_CLOD_LEARN_STA,//开始空调制冷学习
				AIR_CLOD_LEARN_END,//结束空调制冷学习
				AIR_CLOSE_LEARN_STA,//开始空调关闭学习
				AIR_CLOSE_LEARN_END,//结束空调关闭学习
				DEV_ALIVE_PACK,//PC端发送心跳包
				DEV_ALIVE_PACK_BACK,//设备端回应心跳包
		    GET_DEV_TABLE,//pc端获取设备表
			}__485_CMD_;
    /*1.pc端与设备端的简易指令格式:*/
		typedef struct{
			HEAD_TAIL_CMD head;
			char id[20];
			__485_CMD_ cmd;
			HEAD_TAIL_CMD tail;
			uint16_t crc;
		}__485_CMD_Type;
        
    /*2.pc端向设备发送wifi的账号和密码:*/
		typedef struct{
			HEAD_TAIL_CMD head;
			char id[20];
			char name[30];
			char pass[30];
			HEAD_TAIL_CMD tail;
			uint16_t crc;
		}SEND_WIFI_DATA_Type;
	 /*3.pc端向设备发送时间戳:*/
		typedef struct{
			HEAD_TAIL_CMD head;
			uint32_t time_tamp;
			HEAD_TAIL_CMD tail;
			uint16_t crc;
		}SEND_TIME_TAMP_Type;
		
		
	 
	/***************************当前网络时间*************************/ 
	 typedef struct{
		  HEAD_TAIL_CMD head;      
			uint8_t year;
		  uint8_t month;
		  uint8_t day;
		  uint8_t hour;
		  uint8_t minute;
		  uint8_t second;
		  uint8_t weekday;
			HEAD_TAIL_CMD tail;
			uint16_t crc;
	 }NET_TIME;

/***************************当前时间时间戳*************************/
typedef struct{
uint32_t time_date_stamp;
uint32_t oneday_time_stamp;
}NOW_TIME_STAMP_Typedef;

/*********************串口接收整合包********************/
#define USARTx_RCV_MAX_LEN 300
typedef struct{
uint8_t ptr;
uint8_t arry[USARTx_RCV_MAX_LEN];
uint8_t count;
bool rcv_ok;
}USARTx_RCV_Typedef;

#define USART1_RCV_MAX_LEN 1000
typedef struct{
uint8_t ptr;
uint8_t arry[USART1_RCV_MAX_LEN];
uint16_t count;
bool rcv_ok;
}USART1_RCV_Typedef;

/******************传感器参数设定********************/
typedef enum{
READ_DATA = 0X03,
WRITE_ADDR = 0X06
}SHT30_CMD;

typedef enum{
SENSOR_ID = 0X01,
READ_TEMP_ADDR = 0X0002,
READ_HUMI_ADDR = 0X0001,
WRITE_DEV_ADDR = 0X0066
}SHT30_ADDR;
typedef struct{
uint8_t status;
uint8_t id;
uint32_t temp;
uint32_t humi;
}TEMP_HUMI_TypeDef;

typedef struct{
float temp;
float humi;
}LOCAL_TEMP_HUMI_TypeDef;

typedef struct{
uint8_t id;
uint8_t cmd;
uint8_t data_len;
short temp;
short humi;
uint16_t crc;
}RSHT30_READ_TypeDef;


typedef struct{
uint8_t id;
uint8_t cmd;
uint16_t addr;
uint16_t data_len;
uint16_t crc;
}SHT30_WRITE_TypeDef;

/********************获取保种、共育、催青状态结构体*************************/
typedef enum{
BAOZHONG_STA = 0X00,
BAOZHONG_END = 0X01,
GONGYU_STA = 0X02,
GONGYU_END = 0X03,
CUIQING_STA = 0X04,
CUIQING_END = 0X05,
}WORK_WAY_STATUS;   
typedef struct{
HEAD_TAIL_CMD head;      
char id[20];
WORK_WAY_STATUS work_status;
uint16_t reasure_count;	
HEAD_TAIL_CMD tail;
uint16_t crc;
}WORK_WAY_TYEP; 
		
/**********控制各功能的具体结构*************/	
typedef struct{
uint8_t id;
AIR_STATUS air;
RELAY   relay;
uint16_t crc;
}TO_MCU_Typedef;

typedef struct {
	uint16_t len;
	uint16_t data[400];
}AIR_SAVE_TYPE;

typedef struct{
    uint8_t head;
    uint16_t air;
    AIR_SAVE_TYPE data;
    uint8_t tail;
    uint16_t crc;
}AIR_SAVE_CODE_TYPE;
#pragma pack()
/**********空调学习状态结构******/
typedef struct{
bool air_close_learn_status;
bool air_heat_learn_status;
bool air_clod_learn_status;
}AIR_LEARN_STATUS;

extern UPDATE_ALL_TABLE_TYPEDEF  all_table_data;
extern TO_APP_Typedef to_app;
extern NOW_TIME_STAMP_Typedef now_time_stamp;

typedef enum{
IS_AUTO_MODE = 0,
IS_HAND_MODE = !IS_AUTO_MODE, 	
} LCD_WORK_MODE;  
typedef struct{
bool pc_to_mcu_trig;
bool net_to_mcu_trig;
bool lcd_to_mcu_trig;
bool mcu_to_pc_trig;
bool mcu_to_net_trig;
bool mcu_to_lcd_trig;
bool net_trig;
bool write_flash;
bool get_pc_heart;
bool mcu_send_to_app_to_pc;
bool wifi_con;
bool get_wifi_ok;
bool get_wifi_new_con;  
bool senser_is_ok;  
bool pc_wifi_con;   
bool change_id;
bool pc_change_work_status; 
bool net_change_work_status; 
bool update_net_time_trig;
bool update_net_trig_send;
bool lcd_id_mcu;
bool mcu_id_lcd;
bool wifi_pc_mode;
bool lcd_id_recover;
LCD_WORK_MODE lcd_work_mode;   
AIR_LEARN_STATUS pc_air_learn;
AIR_LEARN_STATUS lcd_air_learn;

}TABBLE_TRIG;

/**********************flash存储wifi和工作模式************************/
typedef struct{
WIFI_CFG_Typedef wifi_cfg;
WORK_WAY_TYEP work_way_status;  
}FLASH_STORE;


/***************红外数据格式**********************/
typedef enum {
	RES_OK = 0,
	RES_FAIL,
	RES_INVALID_PARA,
	RES_NO_SPACE
} ResultStatus;

typedef enum {
	STUDY_ORDER = 0,
	EXIT_ORDER,
	SEND_ORDER,
	RECV_ORDER,
	TEN_MS_ORDER,
} OrderEnum;

#define MAXSIZE 					5

typedef struct {
	uint8_t type;
	uint8_t nc;
	uint32_t para1;
} ElemType;

typedef struct{
	ElemType data[MAXSIZE];
	int front;
	int rear;
} Sequeue;

#define MAX_CODE_LENGTH			400

typedef struct {
	uint8_t valid;
	uint8_t nc;
	uint16_t len;
	uint16_t data[MAX_CODE_LENGTH];
} IrCode;



typedef struct {
	IrCode *ir;
} App;


typedef struct{
	bool isDetect;
	bool isSend;
	uint16_t pre_temp;
	uint32_t Ctime;
	uint8_t excute_air;
}INFRA_DETECT;


extern TABBLE_TRIG table_update_trig;


//typedef enum{
//IS_IDEL = 0,
//IS_BUSY = 1,
//}TASK_WRITE_FLAG;
//extern TASK_WRITE_FLAG task_write_flag;


extern UPDATE_ALL_TABLE_TYPEDEF download_update_table ;

extern USART1_RCV_Typedef usart1_rcv;
extern USARTx_RCV_Typedef usart2_rcv;
extern USARTx_RCV_Typedef usart3_rcv;


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern IWDG_HandleTypeDef hiwdg;

extern AIR_LEARN_STATUS air_learn_status;
extern AIR_LEARN_STATUS air_learn_ok_status;
extern TO_MCU_Typedef mcu_control_data;
extern TO_APP_Typedef to_app;
extern __485_CMD_Type __485_send_cmd;
extern uint32_t get_tamp;
extern WORK_WAY_TYEP work_way_status; 
extern FLASH_STORE flash_store;
extern WIFI_CFG_Typedef wifi_cfg;
extern NET_TIME net_time;


extern INFRA_DETECT infraDetect;
extern AIR_SAVE_TYPE save_data;

/***************************配置继电器端口引脚***************************/
#define HEAT_PORT          GPIOB //加热
#define HUMI_PORT          GPIOB //补湿
#define BREATH_PORT        GPIOB //换气
#define SENSITIVE_PORT     GPIOB //感光
#define WIND_PORT          GPIOB //匀风
#define STERILIZE_PORT     GPIOA //消毒

#define LIGHT_PORT         GPIOA //光照

#define DRY_PORT           GPIOA //除湿

#define WIFI_RST_PORT      GPIOB
#define WIFI_EN_PORT       GPIOB
#define IO0_EN_PORT        GPIOC
#define IO2_EN_PORT        GPIOC

 
#define HEAT_PIN           GPIO_PIN_7  //加热
#define HUMI_PIN           GPIO_PIN_6  //补湿
#define BREATH_PIN         GPIO_PIN_5  //换气
#define SENSITIVE_PIN      GPIO_PIN_4 //感光
#define WIND_PIN           GPIO_PIN_3 //匀风
#define STERILIZE_PIN      GPIO_PIN_15 //消毒

#define LIGHT_PIN          GPIO_PIN_12 //光照

#define DRY_PIN            GPIO_PIN_11 //除湿

#define WIFI_RST_PIN       GPIO_PIN_0 
#define WIFI_EN_PIN        GPIO_PIN_1
#define IO0_EN_PIN         GPIO_PIN_15
#define IO2_EN_PIN         GPIO_PIN_14

#define LED_PORT GPIOA
#define LED_PIN	 GPIO_PIN_8 

#define LED_ON	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)

#define LED_Toggle			HAL_GPIO_TogglePin(LED_PORT, LED_PIN)

#define INS_LED_PORT	GPIOC
#define INS_LED_PIN		GPIO_PIN_13

#define wifi_LED_PORT	  GPIOA
#define wifi_LED_PIN		GPIO_PIN_0



#define INS_LED_ON	HAL_GPIO_WritePin(INS_LED_PORT, INS_LED_PIN, GPIO_PIN_RESET)
#define INS_LED_OFF	HAL_GPIO_WritePin(INS_LED_PORT, INS_LED_PIN, GPIO_PIN_SET)

#define LED_Toggle			HAL_GPIO_TogglePin(LED_PORT, LED_PIN)
//App变量初始化
App *AppInit(void);

//获取上下文
App *GetApp(void);

#endif
