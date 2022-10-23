#ifndef __RELAY_H
#define __RELAY_H

#include <stdio.h>
#include <stdbool.h>
#include "config.h"


#define OPEN  1
#define CLOSE 0
#define __OPEN  0
#define __CLOSE 1
#define HEAT_ON            HAL_GPIO_WritePin(HEAT_PORT,HEAT_PIN,GPIO_PIN_SET)
#define HEAT_OFF           HAL_GPIO_WritePin(HEAT_PORT,HEAT_PIN,GPIO_PIN_RESET)
#define HEAT_WRITE(status) HAL_GPIO_WritePin(HEAT_PORT,HEAT_PIN,(GPIO_PinState)status)


#define HUMI_ON             HAL_GPIO_WritePin(HUMI_PORT,HUMI_PIN,GPIO_PIN_SET)
#define HUMI_OFF            HAL_GPIO_WritePin(HUMI_PORT,HUMI_PIN,GPIO_PIN_RESET)
#define HUMI_WRITE(status)  HAL_GPIO_WritePin(HUMI_PORT,HUMI_PIN,(GPIO_PinState)status)

#define BREATH_ON            HAL_GPIO_WritePin(BREATH_PORT,BREATH_PIN,GPIO_PIN_SET)
#define BREATH_OFF           HAL_GPIO_WritePin(BREATH_PORT,BREATH_PIN,GPIO_PIN_RESET)
#define BREATH_WRITE(status) HAL_GPIO_WritePin(BREATH_PORT,BREATH_PIN,(GPIO_PinState)status)

#define DRY_ON               HAL_GPIO_WritePin(DRY_PORT,DRY_PIN,GPIO_PIN_SET)
#define DRY_OFF              HAL_GPIO_WritePin(DRY_PORT,DRY_PIN,GPIO_PIN_RESET)
#define DRY_WRITE(status)    HAL_GPIO_WritePin(DRY_PORT,DRY_PIN,(GPIO_PinState)status)

#define WIND_ON              HAL_GPIO_WritePin(WIND_PORT,WIND_PIN,GPIO_PIN_SET)
#define WIND_OFF             HAL_GPIO_WritePin(WIND_PORT,WIND_PIN,GPIO_PIN_RESET)
#define WIND_WRIET(status)   HAL_GPIO_WritePin(WIND_PORT,WIND_PIN,(GPIO_PinState)status)

#define LIGHT_ON            HAL_GPIO_WritePin(LIGHT_PORT,LIGHT_PIN,GPIO_PIN_SET)
#define LIGHT_OFF           HAL_GPIO_WritePin(LIGHT_PORT,LIGHT_PIN,GPIO_PIN_RESET)
#define LIGHT_WRITE(status) HAL_GPIO_WritePin(LIGHT_PORT,LIGHT_PIN,(GPIO_PinState)status)

#define STERILIZE_ON            HAL_GPIO_WritePin(STERILIZE_PORT,STERILIZE,GPIO_PIN_SET)
#define STERILIZE_OFF           HAL_GPIO_WritePin(STERILIZE_PORT,STERILIZE_PIN,GPIO_PIN_RESET)
#define STERILIZE_WRITE(status) HAL_GPIO_WritePin(STERILIZE_PORT,STERILIZE_PIN,(GPIO_PinState)status)

#define SENSITIVE_ON            HAL_GPIO_WritePin(SENSITIVE_PORT,SENSITIVE,GPIO_PIN_SET)
#define SENSITIVE_OFF           HAL_GPIO_WritePin(SENSITIVE_PORT,SENSITIVE_PIN,GPIO_PIN_RESET)
#define SENSITIVE_WRITE(status) HAL_GPIO_WritePin(SENSITIVE_PORT,SENSITIVE_PIN,(GPIO_PinState)status)

#define COMTX_ON            HAL_GPIO_WritePin(COM_TX_PORT,COM_TX_PIN,GPIO_PIN_SET)
#define COMTX_OFF           HAL_GPIO_WritePin(COM_TX_PORT,COM_TX_PIN,GPIO_PIN_RESET)
#define COMTX_WRITE(status) HAL_GPIO_WritePin(COM_TX_PORT,COM_TX_PIN,(GPIO_PinState)status)

#define COMRX_ON            HAL_GPIO_WritePin(COM_RX_PORT,COM_RX_PIN,GPIO_PIN_SET)
#define COMRX_OFF           HAL_GPIO_WritePin(COM_RX_PORT,COM_RX_PIN,GPIO_PIN_RESET)
#define COMRX_READ				  HAL_GPIO_ReadPin(COM_RX_PORT,COM_RX_PIN)
														

#define WIFI_RST_ON          HAL_GPIO_WritePin(WIFI_RST_PORT,WIFI_RST_PIN,1)
#define WIFI_RST_OFF         HAL_GPIO_WritePin(WIFI_RST_PORT,WIFI_RST_PIN,0)

#define WIFI_EN_ON          HAL_GPIO_WritePin(WIFI_EN_PORT,WIFI_EN_PIN,1)
#define WIFI_EN_OFF         HAL_GPIO_WritePin(WIFI_EN_PORT,WIFI_EN_PIN,0)

#define IO2_EN_ON          HAL_GPIO_WritePin(IO2_EN_PORT,WIFI_EN_PIN,1)
#define IO2_EN_OFF         HAL_GPIO_WritePin(IO2_EN_PORT,WIFI_EN_PIN,0)





void relay_init(void);
void deal_relay_run(void);

ErrorStatus send_air_cmd(AIR_STATUS cmd);//发送空调制冷制热关闭三条指令返回执行成功与否
void learn_air_cmd(AIR_STATUS cmd);//发送学习三种指令
ErrorStatus get_learn_status(AIR_STATUS cmd);//获取三种指令学习成功与否状态

#endif
