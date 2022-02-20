#include "key.h"

uint8_t key_status = KEY_IDLE;

void KEY_Init(void)
{	
	GPIO_InitTypeDef GPIO_StructTypeDef;	
	
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_StructTypeDef.Pin = KEY1_PIN;
	GPIO_StructTypeDef.Mode = GPIO_MODE_INPUT;
	GPIO_StructTypeDef.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_StructTypeDef.Pull = GPIO_NOPULL;
	
	HAL_GPIO_Init(KEY1_PORT, &GPIO_StructTypeDef);
	
	GPIO_StructTypeDef.Pin = KEY2_PIN;
	GPIO_StructTypeDef.Mode = GPIO_MODE_INPUT;
	GPIO_StructTypeDef.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_StructTypeDef.Pull = GPIO_NOPULL;
	
	HAL_GPIO_Init(KEY2_PORT, &GPIO_StructTypeDef);
	
	GPIO_StructTypeDef.Pin = KEY3_PIN;
	GPIO_StructTypeDef.Mode = GPIO_MODE_INPUT;
	GPIO_StructTypeDef.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_StructTypeDef.Pull = GPIO_NOPULL;
	
	HAL_GPIO_Init(KEY3_PORT, &GPIO_StructTypeDef);
}

uint8_t KEY_Scan(void)
{
	int a  = 0;
	if(KEY1_READ == KEY_DOWN)
	{
		HAL_Delay(10);
		
		while(KEY1_READ  == KEY_DOWN && a < 1000){
					HAL_Delay(10);
					a++;
		}
		
		if( a > 100 )
			return KEY1_KONG_CLICK;
		return KEY1_CLICK;
	}
	
	else if(KEY2_READ == KEY_DOWN)
	{
		HAL_Delay(10);
		while(KEY2_READ  == KEY_DOWN && a < 1000){
					HAL_Delay(10);
					a++;
		}
		
		if( a > 100 )
			return KEY2_KONG_CLICK;
		return KEY2_CLICK;
	}
	
	else if(KEY3_READ == KEY_DOWN)
	{
		HAL_Delay(10);
		while(KEY3_READ  == KEY_DOWN && a < 1000){
					HAL_Delay(10);
					a++;
		}
		
		if( a > 100 )
			return KEY3_KONG_CLICK;
		return KEY3_CLICK;
	}
		
	return KEY_IDLE;

}
