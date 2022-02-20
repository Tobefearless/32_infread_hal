#ifndef __KEY_H
#define __KEY_H

#include "stm32f1xx.h"

#define KEY1_PIN GPIO_PIN_12
#define KEY1_PORT GPIOB

#define KEY2_PIN GPIO_PIN_13
#define KEY2_PORT GPIOB

#define KEY3_PIN GPIO_PIN_14
#define KEY3_PORT GPIOB

#define KEY1_READ HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN)
#define KEY2_READ HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN)
#define KEY3_READ HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN)

#define KEY_DOWN GPIO_PIN_RESET
#define KEY_UP GPIO_PIN_SET

typedef enum  
{
	KEY_IDLE = 0,
	KEY1_CLICK,
	KEY1_DOUBLE_CLICK,
	KEY1_KONG_CLICK,
	
	KEY2_CLICK,
	KEY2_DOUBLE_CLICK,
	KEY2_KONG_CLICK,
	
	KEY3_CLICK,
	KEY3_DOUBLE_CLICK,
	KEY3_KONG_CLICK,
}KeyType;

void KEY_Init(void);

uint8_t KEY_Scan(void);
extern uint8_t key_status;

#endif
