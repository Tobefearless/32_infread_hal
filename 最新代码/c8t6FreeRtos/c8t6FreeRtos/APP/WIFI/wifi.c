#include "wifi.h"
#include "string.h"
#include "time.h"
WIFI_CFG_Typedef wifi_cfg = {
	.enable = false,
	.wif_conect_status = false,
};

char *MQTTCONN 		= "AT+MQTTCONN=0,\"1.14.26.239\",1883,0\r\n";
char *MQTTUSERCFG = "AT+MQTTUSERCFG=0,1,\"uuu\",\"admin\",\"czy123456\",0,0,\"\"\r\n";
char *CWMODE="AT+CWMODE=1\r\n";
char *RST="AT+RST\r\n";
char *MQTTCLEAN="AT+MQTTCLEAN=0\r\n";

ErrorStatus esp826607wifi_connect(void)
{
	memset(wifi_cfg.mqtt_con,0,sizeof(wifi_cfg.mqtt_con));

	printf("AT+CWMODE=1\r\n");
	vTaskDelay(500);
	if(wifi_cfg.enable == true){
			printf("AT\r\n");
			vTaskDelay(500);
			printf("AT\r\n");
			vTaskDelay(500);
		//HAL_GPIO_WritePin(wifi_LED_PORT,wifi_LED_PIN,!HAL_GPIO_ReadPin(wifi_LED_PORT, wifi_LED_PIN));
		vTaskDelay(500);
		printf("AT+MQTTCLEAN=0\r\n");
//		if(sendCommand("AT+MQTTCLEAN=0\r\n", "OK", 10,1) != SUCCESS){
//					return ERROR;
//		}
		WIFI_STOP();
		
		vTaskDelay(500);
		WIFI_START();
		
		memset(wifi_cfg.wifi_con,0,sizeof(wifi_cfg.wifi_con));
		memset(wifi_cfg.mqtt_con,0,sizeof(wifi_cfg.mqtt_con));
		
		memset(wifi_cfg.mqtt_sub,0,sizeof(wifi_cfg.mqtt_sub));
		memset(wifi_cfg.mqtt_pub,0,sizeof(wifi_cfg.mqtt_pub));
		
		strcat(wifi_cfg.wifi_con,"AT+CWJAP=\"");
		strcat(wifi_cfg.wifi_con,wifi_cfg.name);
		strcat(wifi_cfg.wifi_con,"\",\"");
		strcat(wifi_cfg.wifi_con,wifi_cfg.pasw);
		strcat(wifi_cfg.wifi_con,"\"\r\n");
		
		if(sendCommand(wifi_cfg.wifi_con, "OK", 10,1) != SUCCESS){
					return ERROR;
		}
		vTaskDelay(2000);
		
		strcat(wifi_cfg.id,"AT+MQTTUSERCFG=0,1,\"");
		strcat(wifi_cfg.id,all_table_data.id);
		strcat(wifi_cfg.id,"\",\"admin\",\"czy123456\",0,0,\"\"\r\n");
		
		if(sendCommand(wifi_cfg.id, "OK", 10,1) != SUCCESS){
					return ERROR;
		}
				vTaskDelay(2000);
		printf("AT+MQTTCONNCFG=0,60,1,\"123\",\"123\",0,0\r\n");
		vTaskDelay(500);
//		strcat(wifi_cfg.id,wifi_cfg.pasw);
//		strcat(wifi_cfg.id,"\r\n");
//		if(sendCommand(MQTTUSERCFG, "OK", 10,1) != SUCCESS){
//					return ERROR;
//		}


		
		if(sendCommand(MQTTCONN, "OK", 10,1) != SUCCESS){
				return ERROR;
		}
//		wifi_LED_ON;
		wifi_cfg.wif_conect_status = true;
		
		wifi_cfg.wif_conect_status = true;
		wifi_cfg.enable = false;
		table_update_trig.get_wifi_new_con = false;
		return SUCCESS;
		
	}
	vTaskDelay(2000);
		
		if(sendCommand(MQTTCONN, "OK", 10,2) != SUCCESS){
				return ERROR;
		}
		vTaskDelay(2000);
		wifi_cfg.wif_conect_status = false;
		wifi_cfg.enable = true;
		if(sendCommand("AT+CWJAP?\r\n", "OK", 1,1) == SUCCESS){
			 wifi_cfg.wif_conect_status = false;
		   wifi_cfg.enable = true;
		}
		
		return SUCCESS;
		
	return ERROR;
}

ErrorStatus esp826601_wifi_connect(void)
{
	WIFI_START();
	HAL_GPIO_WritePin(IO0_EN_PORT,IO0_EN_PIN,GPIO_PIN_SET);
	memset(wifi_cfg.mqtt_con,0,sizeof(wifi_cfg.mqtt_con));
	if(wifi_cfg.enable == true){
		
//		WIFI_STOP();
//		vTaskDelay(500);
//		WIFI_START();
//		vTaskDelay(1000);
		memset(wifi_cfg.wifi_con,0,sizeof(wifi_cfg.wifi_con));
		memset(wifi_cfg.mqtt_con,0,sizeof(wifi_cfg.mqtt_con));
		
		memset(wifi_cfg.mqtt_sub,0,sizeof(wifi_cfg.mqtt_sub));
		memset(wifi_cfg.mqtt_pub,0,sizeof(wifi_cfg.mqtt_pub));
		
		strcat(wifi_cfg.wifi_con,"AT+AP=");
		strcat(wifi_cfg.wifi_con,wifi_cfg.name);
		strcat(wifi_cfg.wifi_con,",");
		strcat(wifi_cfg.wifi_con,wifi_cfg.pasw);
		strcat(wifi_cfg.wifi_con,"\r\n");
		
		strcat(wifi_cfg.mqtt_con,"AT+MQTTCLOUD=1.14.26.239,1883,admin,czy123456\r\n");
		
		if(sendCommand(wifi_cfg.wifi_con, "AP CONNECTED", 10,1) == SUCCESS){
			wifi_cfg.wif_conect_status = true;
		}else{
			wifi_cfg.wif_conect_status = false;
			table_update_trig.get_wifi_new_con = true;
			return ERROR;
		}
		
		if(sendCommand(wifi_cfg.mqtt_con, "ok", 5,1) == SUCCESS){
			wifi_cfg.wif_conect_status = true;
			wifi_cfg.enable = false;
			table_update_trig.get_wifi_new_con = false;
			memset(wifi_cfg.mqtt_con,0,sizeof(wifi_cfg.mqtt_con));
			return SUCCESS;
		}
		
		
	}
	vTaskDelay(1000);
	strcat(wifi_cfg.mqtt_con,"AT+MQTTCLOUD=1.14.26.239,1883,admin,czy123456\r\n");
	//HAL_UART_Transmit_IT(&huart2,wifi_cfg.mqtt_con,strlen((char*)wifi_cfg.mqtt_con)+1);
	
	if(sendCommand(wifi_cfg.mqtt_con, "ok", 5,1) == SUCCESS){
		wifi_cfg.wif_conect_status = true;
		wifi_cfg.enable = false;
		vTaskDelay(1000);
		if(sendCommand("AT+GAPSTATUS\r\n", "AT+INFO=failed", 1,1) == SUCCESS){
			 wifi_cfg.wif_conect_status = false;
		   wifi_cfg.enable = true;
		 }else{
			return SUCCESS;
		 }
		 
	}
	
	return ERROR;
}

void wifi_init(void)
{
GPIO_InitTypeDef GPIO_Init;
HAL_GPIO_WritePin(IO0_EN_PORT,IO0_EN_PIN,GPIO_PIN_SET);
GPIO_Init.Pin = WIFI_RST_PIN | WIFI_EN_PIN | IO0_EN_PIN | IO2_EN_PIN;
GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
HAL_GPIO_Init(WIFI_RST_PORT, &GPIO_Init);
HAL_GPIO_Init(GPIOC, &GPIO_Init);
WIFI_START();

}

ErrorStatus sendCommand(char *Command, char *Response, unsigned long Timeout,uint8_t send_count)
{
	char Time_Cont = 0;
	memset( usart3_rcv.arry,0,sizeof(usart3_rcv.arry));
	for(;send_count-->0;){
	  usart3_rcv.rcv_ok = false;
		Time_Cont = 0;
		printf(Command); 		//∑¢ÀÕwifi÷∏¡Ó
		while (Time_Cont < Timeout )
		{
			Time_Cont++;
			vTaskDelay(1000);
			if(usart3_rcv.rcv_ok == true){
				usart3_rcv.rcv_ok = false;

				if (strstr((char*)(usart3_rcv.arry), Response) != NULL)	
					
					return SUCCESS;
			}
		}
	}
	
	return ERROR;
}
