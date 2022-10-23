#include "relay.h"
#include "string.h"
#include "infrared.h"
TO_MCU_Typedef mcu_control_data ;
AIR_LEARN_STATUS air_learn_status = {false,false,false};
AIR_LEARN_STATUS air_learn_ok_status = {false,false,false};

static App *app;
void IR_Recv_Callback(uint8_t status, IrRecv *recv);

AIR_SAVE_TYPE save_data;
//AIR_SAVE_TYPE read_data;





ErrorStatus send_air_cmd(AIR_STATUS cmd)
{
	
#if 1
		uint8_t i;
		memset(&save_data,0,sizeof(save_data));
		readFlashTest((uint16_t*)&save_data,sizeof(save_data)/2,cmd);
		if( save_data.len > 50 &&  save_data.len < 500 ){
						app->ir->valid = 1;
						app->ir->len = save_data.len;
						memcpy(app->ir->data, save_data.data, save_data.len * sizeof(uint16_t));
						for( i = 0 ; i < 2 ; i++ ){
								IrSend_Start(app->ir);
								vTaskDelay(1000);
						}
						return SUCCESS;
				}
		else{
				return ERROR;
		}

		
#else
	uint8_t time_out = 0;
	usart2_rcv.rcv_ok = false;
	for(uint8_t i = 0;i<1;i++){
	 HAL_UART_Transmit(&huart3,(uint8_t*)&Send_Air_Cmd[cmd], sizeof(Send_Air_Cmd[cmd]),0xffff);
	 vTaskDelay(90);
	}
	
	while(usart3_rcv.rcv_ok == false){
		vTaskDelay(10);
		if(time_out++>50)
			return ERROR;
	}
	
	if(memcmp(usart3_rcv.arry,Get_Send_Status,usart3_rcv.count)==0)
      return SUCCESS;
  else
      return ERROR;
	
  return SUCCESS;
#endif
}




void learn_air_cmd(AIR_STATUS cmd) 
{
	
	IrRecv_Start(IR_Recv_Callback);
	
	//uint8_t time_out = 0;
//	bool* status;
//	switch(cmd){
//		case  AIR_CLOSE: status =   &(air_learn_status.air_close_learn_status);
//		      break;
//		case  AIR_HEAT: status =   &(air_learn_status.air_heat_learn_status);
//		      break;
//		case  AIR_COLD: status =   &(air_learn_status.air_clod_learn_status);
//		      break;
//	}
//	*status = false;
//	HAL_UART_Transmit(&huart3,(uint8_t*)&Learn_Air_Cmd[cmd], sizeof(Learn_Air_Cmd[cmd]),0xffff);
}




//¼ÌµçÆ÷³õÊ¼»¯
void relay_init(void)
{
	mcu_control_data.air = (AIR_STATUS)CLOSE;
	mcu_control_data.relay.relay_vale = 0x00;
	GPIO_InitTypeDef GPIO_Init;
	
  GPIO_Init.Speed = GPIO_SPEED_HIGH;
	GPIO_Init.Pin = HEAT_PIN;
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(HEAT_PORT, &GPIO_Init);
	HEAT_OFF;
	
	GPIO_Init.Pin = HUMI_PIN;
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(HUMI_PORT, &GPIO_Init);
	HUMI_OFF;
	
	GPIO_Init.Pin = DRY_PIN;
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(DRY_PORT, &GPIO_Init);
	DRY_OFF;
	
	GPIO_Init.Pin = BREATH_PIN;
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(BREATH_PORT, &GPIO_Init);
	BREATH_OFF;
	
	GPIO_Init.Pin = WIND_PIN;
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(WIND_PORT, &GPIO_Init);
	WIND_OFF;
	
	GPIO_Init.Pin = LIGHT_PIN;
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(LIGHT_PORT, &GPIO_Init);
	LIGHT_OFF;
	
	GPIO_Init.Pin = STERILIZE_PIN;
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(STERILIZE_PORT, &GPIO_Init);
	STERILIZE_OFF;
	
	GPIO_Init.Pin = SENSITIVE_PIN;
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(SENSITIVE_PORT, &GPIO_Init);
	SENSITIVE_OFF;
	
	GPIO_Init.Pin = SENSITIVE_PIN;
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(SENSITIVE_PORT, &GPIO_Init);
	SENSITIVE_OFF;
	app = AppInit();
	
	
}



void deal_relay_run(void)
{
	static uint8_t status = 0XFF;
	
	HEAT_WRITE(mcu_control_data.relay.relay.heat);
	HUMI_WRITE(mcu_control_data.relay.relay.humi);
	BREATH_WRITE(mcu_control_data.relay.relay.breath);
	DRY_WRITE(mcu_control_data.relay.relay.dry);
	WIND_WRIET(mcu_control_data.relay.relay.wind);
	LIGHT_WRITE(mcu_control_data.relay.relay.light);
	STERILIZE_WRITE(mcu_control_data.relay.relay.sterilize);
	SENSITIVE_WRITE(mcu_control_data.relay.relay.sensitive);
	
	if(infraDetect.isSend || mcu_control_data.air != status){
		
		status = mcu_control_data.air;
		if( infraDetect.isSend ){
				status = infraDetect.excute_air;
				//infraDetect.excute_air = 0xff;
		}
				
		send_air_cmd(mcu_control_data.air);
		infraDetect.isSend = false;
	}
	
}

void IR_Recv_Callback(uint8_t status, IrRecv *recv)
{
	
	uint8_t num = 0;
	if (status != RES_OK) {
		if( air_learn_status.air_heat_learn_status == true ){
				air_learn_status.air_heat_learn_status = false;
				LED_OFF;
		}
		else if( air_learn_status.air_close_learn_status == true ){
				air_learn_status.air_close_learn_status = false;
				LED_OFF;
		}
		else if( air_learn_status.air_clod_learn_status == true ){
				air_learn_status.air_clod_learn_status = false;
				LED_OFF;
		}
		return;
	}
	memset(&save_data,0,sizeof(save_data));
	app->ir->valid = 1;
	app->ir->len = recv->len;
	
	save_data.len = recv->len;
	memcpy(save_data.data, recv->data, recv->len * sizeof(uint16_t));
	memcpy(app->ir->data, recv->data, recv->len * sizeof(uint16_t));
	
	if( air_learn_status.air_close_learn_status == true ){
				air_learn_status.air_close_learn_status    = false;
				air_learn_ok_status.air_close_learn_status = true;
				LED_OFF;
				num = AIR_CLOSE;
	}
	else if( air_learn_status.air_clod_learn_status == true){
				air_learn_status.air_clod_learn_status   = false;
				air_learn_ok_status.air_clod_learn_status = true;
				LED_OFF;
				num = AIR_COLD;
	}
	else if( air_learn_status.air_heat_learn_status == true ){
				air_learn_status.air_heat_learn_status    = false;
				air_learn_ok_status.air_heat_learn_status = true;
				LED_OFF;
				num = AIR_HEAT;
	}

	writeFlashTest((uint16_t*)&save_data,sizeof(save_data)/2,num);
	LED_OFF;
}
