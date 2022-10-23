#ifndef __WIFI_H
#define __WIFI_H 
#include "config.h" 
#include <stdbool.h>
#include <stdio.h>

#define WIFI_START() {HAL_GPIO_WritePin(WIFI_EN_PORT,WIFI_EN_PIN,GPIO_PIN_SET);\
											HAL_GPIO_WritePin(IO0_EN_PORT,IO0_EN_PIN,GPIO_PIN_SET);\
											HAL_GPIO_WritePin(IO2_EN_PORT,IO2_EN_PIN,GPIO_PIN_SET);}

#define WIFI_STOP() {HAL_GPIO_WritePin(WIFI_EN_PORT,WIFI_EN_PIN,GPIO_PIN_SET);\
											HAL_GPIO_WritePin(IO0_EN_PORT,IO0_EN_PIN,GPIO_PIN_SET);\
											HAL_GPIO_WritePin(IO2_EN_PORT,IO2_EN_PIN,GPIO_PIN_SET);}


#define wifi_LED_OFF	 HAL_GPIO_WritePin(wifi_LED_PORT, wifi_LED_PIN, GPIO_PIN_RESET)
#define wifi_LED_ON	HAL_GPIO_WritePin(wifi_LED_PORT, wifi_LED_PIN, GPIO_PIN_SET)
#define wifi_LED_Toggle			HAL_GPIO_TogglePin(wifi_LED_PORT,wifi_LED_PIN)
#define wifi_Conect_Status_LED 		if(wifi_cfg.enable == true || wifi_cfg.wif_conect_status == false){	\
																				HAL_GPIO_TogglePin(wifi_LED_PORT,wifi_LED_PIN);				\
																	}	

ErrorStatus esp826607wifi_connect(void);
ErrorStatus esp826601_wifi_connect(void);
ErrorStatus sendCommand(char *Command, char *Response, unsigned long Timeout,uint8_t send_count);
#endif
