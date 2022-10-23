#include "modbus.h"
#include <string.h>
#include <math.h>


/***********************换气参数周期****************/
TIME_SHOW_ADDR breath_show[8]={
{.time_start_addr = FIRST_AERATION_START_ADDR,.time_end_addr = FIRST_AERATION_TIME_ADDR},			
{.time_start_addr = SECOND_AERATION_START_ADDR,.time_end_addr = SECOND_AERATION_TIME_ADDR},
{.time_start_addr = THIRD_AERATION_START_ADDR,.time_end_addr = THIRD_AERATION_TIME_ADDR},
{.time_start_addr = FOURTH_AERATION_START_ADDR,.time_end_addr = FOURTH_AERATION_TIME_ADDR},
{.time_start_addr = FIFTH_AERATION_START_ADDR,.time_end_addr = FIFTH_AERATION_TIME_ADDR},
{.time_start_addr = SIXTH_AERATION_START_ADDR,.time_end_addr = SIXTH_AERATION_TIME_ADDR},
{.time_start_addr = SEVENTH_AERATION_START_ADDR,.time_end_addr = SEVENTH_AERATION_TIME_ADDR},
{.time_start_addr = EIGHTH_AERATION_START_ADDR,.time_end_addr = EIGHTH_AERATION_TIME_ADDR}
};

/********************感光参数周期*******************/
TIME_SHOW_ADDR sensitive_show[3]={
{.time_start_addr = FIRST_SENSITIVE_START_ADDR ,.time_end_addr = FIRST_SENSITIVE_END_ADDR},
{.time_start_addr = SECOND_SENSITIVE_START_ADDR ,.time_end_addr = SECOND_SENSITIVE_END_ADDR},
{.time_start_addr = THIRD_SENSITIVE_START_ADDR ,.time_end_addr = THIRD_SENSITIVE_END_ADDR}
};

/********************光照参数周期*******************/
TIME_SHOW_ADDR light_show[3]={
{.time_start_addr = FIRST_LIGHT_START_ADDR,.time_end_addr = FIRST_LIGHT_END_ADDR},
{.time_start_addr = SECOND_LIGHT_START_ADDR,.time_end_addr = SECOND_LIGHT_END_ADDR},
{.time_start_addr = THIRD_LIGHT_START_ADDR,.time_end_addr = THIRD_LIGHT_END_ADDR}
};

/**********************消毒参数周期*****************/
TIME_SHOW_ADDR sterilize_show[4]={
{.time_start_addr = FIRST_STERILIZE_START_ADDR,.time_end_addr = FIRST_STERILIZE_END_ADDR},
{.time_start_addr = SECOND_STERILIZE_START_ADDR,.time_end_addr = SECOND_STERILIZE_END_ADDR},
{.time_start_addr = THIRD_STERILIZE_START_ADDR,.time_end_addr = THIRD_STERILIZE_END_ADDR},
{.time_start_addr = FOURTH_STERILIZE_START_ADDR,.time_end_addr = FOURTH_STERILIZE_END_ADDR}
};

/******************温湿度传感器校准****************/
SENSOR_SHOW_ADDR sensor_show[4] = {
{.sensor_id_addr= FIRST_SENSOR_ADDR,.sigle_sensor_temp_addr = FIRST_REAL_TIME_TEMP_VALUE_ADDR,
.sigle_sensor_humi_addr = FIRST_REAL_TIME_HUMI_VALUE_ADDR,.sigle_sensor_adjust_temp_addr = FIRST_TEMP_CALIBRATION_VALUE_ADDR,
.sigle_sensor_adjust_humi_addr = FIRST_HUMI_CALIBRATION_VALUE_ADDR},

{.sensor_id_addr= SECOND_SENSOR_ADDR,.sigle_sensor_temp_addr = SECOND_REAL_TIME_TEMP_VALUE_ADDR,
.sigle_sensor_humi_addr = SECOND_REAL_TIME_HUMI_VALUE_ADDR,.sigle_sensor_adjust_temp_addr = SECOND_TEMP_CALIBRATION_VALUE_ADDR,
.sigle_sensor_adjust_humi_addr = SECOND_HUMI_CALIBRATION_VALUE_ADDR},

{.sensor_id_addr= THIRD_SENSOR_ADDR,.sigle_sensor_temp_addr = THIRD_REAL_TIME_TEMP_VALUE_ADDR,
.sigle_sensor_humi_addr = THIRD_REAL_TIME_HUMI_VALUE_ADDR,.sigle_sensor_adjust_temp_addr = THIRD_TEMP_CALIBRATION_VALUE_ADDR,
.sigle_sensor_adjust_humi_addr = THIRD_HUMI_CALIBRATION_VALUE_ADDR},

{.sensor_id_addr= FOURTH_SENSOR_ADDR,.sigle_sensor_temp_addr = FOURTH_REAL_TIME_TEMP_VALUE_ADDR,
.sigle_sensor_humi_addr = FOURTH_REAL_TIME_HUMI_VALUE_ADDR,.sigle_sensor_adjust_temp_addr = FOURTH_TEMP_CALIBRATION_VALUE_ADDR,
.sigle_sensor_adjust_humi_addr = FOURTH_HUMI_CALIBRATION_VALUE_ADDR}
};



time_t read_local_time(void)
{
	struct tm stm;
	uint8_t time_out = 0;
  static uint8_t buf[6] = {0xEE,0x82,0xFF,0xFC,0xFF,0xFF};
	usart2_rcv.rcv_ok = false;
	HAL_UART_Transmit(&huart2,buf,sizeof(buf),0xfffff);
	while(usart2_rcv.rcv_ok == false){
		vTaskDelay(50);
		if(time_out++>5)
			return 0;
	}
	
	if(usart2_rcv.arry[0]== 0xEE 
		 && usart2_rcv.arry[1] == 0XF7 ){
		 usart2_rcv.rcv_ok = false;
		 stm.tm_year = 2000+ (usart2_rcv.arry[2]/16)*10 + (usart2_rcv.arry[2]%16) - 1900;
		 stm.tm_mon =  (usart2_rcv.arry[3]/16)*10 + (usart2_rcv.arry[3]%16)-1;
		 stm.tm_mday = (usart2_rcv.arry[5]/16)*10 + (usart2_rcv.arry[5]%16);
		 stm.tm_hour = (usart2_rcv.arry[6]/16)*10 + (usart2_rcv.arry[6]%16);
		 stm.tm_min = (usart2_rcv.arry[7]/16)*10 + (usart2_rcv.arry[7]%16);
		 stm.tm_sec = (usart2_rcv.arry[8]/16)*10 + (usart2_rcv.arry[8]%16);
		 return mktime(&stm); 
	}

 return 0;
}

void write_txt_data(uint16_t write_fun_cmd, uint32_t addr, char* data)
{
	static TXT_SEND_Typedef txt_send;
	memset(&txt_send,0,sizeof(TXT_SEND_Typedef));
	txt_send.head = HEAD;
	txt_send.fun_cmd = swap_16(write_fun_cmd);
	txt_send.addr = swap_32(addr);
	memcpy(txt_send.data,data,sizeof(txt_send.data));
	txt_send.tail = swap_32(TAIL);
	UNUSED(huart2);
	HAL_UART_Transmit(&huart2,(uint8_t*)&txt_send,sizeof(TXT_SEND_Typedef),0xffff);
//	if( strcmp(data,"共育运行") == 0 )
//	{
//		//printf("保种停止\r\n");
//		HAL_UART_Transmit(&huart3,(uint8_t*)&txt_send,sizeof(TXT_SEND_Typedef),0xffff);
//	}
}

char* read_txt_data(uint16_t get_fun_cmd, uint16_t read_fun_cmd, uint32_t addr)
{
	static char buf[30];
	TXT_READ_Typedef rcv_data;
	static TXT_SEND_CMD_Typedef send_cmd;
	uint8_t time_out = 0;
	memset(usart2_rcv.arry,0,sizeof(usart2_rcv.arry));
	memset(buf,0,sizeof(buf));
	usart2_rcv.rcv_ok = false;
	send_cmd.head = HEAD;
	send_cmd.fun_cmd = swap_16(get_fun_cmd);
	send_cmd.addr = swap_32(addr);
	send_cmd.tail = swap_32(TAIL);
	HAL_UART_Transmit(&huart2,(uint8_t*)&send_cmd,sizeof(TXT_SEND_CMD_Typedef),0xffff);
	while(usart2_rcv.rcv_ok == false){
		vTaskDelay(10);
		
		if(time_out++>50)
			return NULL;
	}
	memcpy(&rcv_data,usart2_rcv.arry,sizeof(TXT_READ_Typedef));
  if( swap_16(read_fun_cmd) == rcv_data.fun_cmd && swap_32(addr) == rcv_data.addr){
		uint8_t len = strlen(&rcv_data.data[1]);
		memcpy(buf,&rcv_data.data[1],len);
		return buf;
	}
	return NULL;
}
void write_hex_data(uint16_t write_fun_cmd, uint32_t addr, uint8_t data)
{
	HEX_SEND_Typedef hex_send;
	hex_send.head = HEAD;
	hex_send.fun_cmd = swap_16(write_fun_cmd);
	hex_send.addr = swap_32(addr);
	hex_send.data = data;
	hex_send.tail = swap_32(TAIL);
	HAL_UART_Transmit(&huart2,(uint8_t*)&hex_send,sizeof(HEX_SEND_Typedef),0xffff);
}
SWITCH_STATUS read_hex_data(uint16_t get_fun_cmd, uint16_t read_fun_cmd, uint32_t addr)
{
	static SWITCH_STATUS data;
	HEX_READ_Typedef rcv_data;
	static HEX_SEND_CMD_Typedef send_cmd;
	uint8_t time_out = 0;
	data = NO_DATA;
	usart2_rcv.rcv_ok = false;
	send_cmd.head = HEAD;
	send_cmd.fun_cmd = swap_16(get_fun_cmd);
	send_cmd.addr = swap_32(addr);
	send_cmd.tail = swap_32(TAIL);
	
	HAL_UART_Transmit(&huart2,(uint8_t*)&send_cmd,sizeof(HEX_SEND_CMD_Typedef),0xffff);
	while(usart2_rcv.rcv_ok == false){
		vTaskDelay(10);
		if(time_out++>5)
			return NO_DATA;
	}
	memcpy(&rcv_data,usart2_rcv.arry,sizeof(HEX_READ_Typedef));
	if(rcv_data.addr == swap_32(addr) 
		&& swap_16(read_fun_cmd) == rcv_data.fun_cmd){
		data = (SWITCH_STATUS)rcv_data.data[2];
		return data;
	}
	return	NO_DATA;
}

time_t dateTime_to_secs(char* dataTime)
{

	struct tm stm;  
	int iY, iM, iD, iH, iMin, iS;  

	memset(&stm,0,sizeof(stm));  

	iY = atoi(dataTime);  
	iM = atoi(dataTime+5);  
	iD = atoi(dataTime+8);  
	iH = atoi(dataTime+11);  
	iMin = atoi(dataTime+14);  
	iS = atoi(dataTime+17);  

	stm.tm_year=iY-1900;  
	stm.tm_mon=iM-1;  
	stm.tm_mday=iD;  
	stm.tm_hour=iH;  
	stm.tm_min=iMin;  
	stm.tm_sec=iS;  
  

	return mktime(&stm);
	
		
}	
	
	
time_t time_to_secs(char* time)
{
	int iH, iMin;
	time_t mk_time;
	if(strstr(time,":")==NULL)
		return 0;
	iH = atoi(time);
	if(time[1]==':')
	 iMin = atoi(time+2);
	if(time[2] == ':')
		iMin = atoi(time+3);
	mk_time = iH*3600+iMin*60;
	return mk_time;
}
char* secs_to_dateTime(time_t secs)
{
	static char nowtime[24]; 
  struct tm *lt	;
	lt = localtime(&secs);
	memset(nowtime, 0, sizeof(nowtime)); 
	strftime(nowtime, 24, "%Y-%m-%d %H:%M:%S", lt); 
	return nowtime; 
}

char* secs_to_date(time_t secs)
{
	static char nowtime[24]; 
  struct tm *lt	;
	secs += 3600*24;
	lt = localtime(&secs);
	memset(nowtime, 0, sizeof(nowtime)); 
  strftime(nowtime, 24, "%m-%d", lt);  
	return nowtime; 
}

char* secs_to_time(time_t secs)
{
	int iH, iMin;
	iH = secs/3600;
	iMin = (secs%3600)/60;
	static char buf[20];
	memset(buf, 0, sizeof(buf));
	sprintf(buf,"%02d:%02d",iH,iMin);
	return buf;
}

static void update_net_time(void)
{
	uint8_t buf[13] = {0xEE,0x81,0x58,0x35,0x20,0x14,0x05,0x05,0x21,0xFF,0xFC,0xFF,0xFF};
	if(net_time.second <= 0x56)
		net_time.second+=1;
	buf[8] = net_time.year;
	buf[7] = net_time.month;
	buf[6] = net_time.weekday;
	buf[5] = net_time.day;
	buf[4] = net_time.hour;
	buf[3] = net_time.minute;
	buf[2] = net_time.second;
	if(table_update_trig.update_net_time_trig == true){
		HAL_UART_Transmit(&huart2,(uint8_t*)&buf,sizeof(buf),0xffff);
		table_update_trig.update_net_time_trig = false;
	}
}

/*************************************业务处理部分************************************/


void deal_lcd_show(void)
{
	char buf[20] = {0};
	
	//创建一个临时的参数结构体
	static UPDATE_ALL_TABLE_TYPEDEF table_data;
	
	//继电器状态结构体
	//static SWITCH_STATUS status;
	//一个指针
	static char*p = NULL;
	//读取屏幕上的时间
	time_t tm = read_local_time();
	
	//如果时间b
	if(tm != 0){
		get_tamp = tm;
	}
	
	devid_to_lcd_txt(all_table_data.id);
	size_t temp = xPortGetFreeHeapSize();
	if(table_update_trig.senser_is_ok){
		sprintf(buf,"%2.1f",(float)(to_app.temp)/10);
		//sprintf(buf,"%d",temp);
		temp_to_lcd_txt(buf);
		sprintf(buf,"%2.1f",(float)(to_app.humi)/10);
		humi_to_lcd_txt(buf);
  }else{
		temp_to_lcd_txt("null");
		humi_to_lcd_txt("null");
	}
	
	update_net_time();
	
	if( table_update_trig.lcd_id_mcu == true ){
			p = id_from_lcd_txt();
			if( p != NULL ){
				set_default_parameters(p);
				recover_id_to_lcd_hex(0);
				table_update_trig.lcd_id_mcu = false;
			}		
			
	}
	else if( table_update_trig.mcu_id_lcd == true){
			write_id_data1();
			write_id_data();
		  recover_id_to_lcd_hex(0);			
			table_update_trig.mcu_id_lcd = false;		
	}
	else if(table_update_trig.lcd_id_recover == true){
		p = id_from_lcd_txt();
		if(p != NULL){
			set_default_parameters(p);
			id_change_ok_to_lcd_hex(0);			
		}
		table_update_trig.lcd_id_recover = false;
	}
	
	
	if(table_update_trig.lcd_air_learn.air_clod_learn_status == true){
		air_learn_status_to_lcd_hex(3);
		vTaskDelay(100);
		air_learn_status_to_lcd_hex(3);
		table_update_trig.lcd_air_learn.air_clod_learn_status = false;
	}
	
	if(table_update_trig.lcd_air_learn.air_close_learn_status == true){
		air_learn_status_to_lcd_hex(3);
		vTaskDelay(100);
		air_learn_status_to_lcd_hex(3);
		table_update_trig.lcd_air_learn.air_close_learn_status = false;
	}
	
	if(table_update_trig.lcd_air_learn.air_heat_learn_status == true){
		air_learn_status_to_lcd_hex(3);
		vTaskDelay(100);
		air_learn_status_to_lcd_hex(3);
		table_update_trig.lcd_air_learn.air_heat_learn_status = false;
	}
	
	if(table_update_trig.wifi_con == true){
		 wifi_cfg.wif_conect_status = false;
		 table_update_trig.get_wifi_new_con = false;
		 bool flag1 = false,flag2 = false;
			p = wifi_name_from_lcd_txt();  
      if(p != NULL){
				memset(wifi_cfg.name,0,sizeof(wifi_cfg.name));
				strcpy(wifi_cfg.name,p);
				flag1 = true;
			}	
      p =wifi_passwd_from_lcd_txt(); 
			if(p != NULL){
				memset(wifi_cfg.pasw,0,sizeof(wifi_cfg.pasw));
				strcpy(wifi_cfg.pasw,p);
				flag2 = true;
			}
			
			if(flag1 == true && flag2 == true){
				wifi_cfg.enable = true;
				table_update_trig.get_wifi_ok = true;
				table_update_trig.wifi_con = false;
				flash_store.wifi_cfg = wifi_cfg;
				flash_store.wifi_cfg.enable = true;
				flash_store.wifi_cfg.wif_conect_status = false;
				flash_store.work_way_status = work_way_status;
				flash__write((uint16_t*)&flash_store,sizeof(FLASH_STORE));
			}
	}
	
	if(wifi_cfg.wif_conect_status == true && table_update_trig.get_wifi_ok == true){
		wifi_connect_to_lcd_hex(1);
	}
	if(table_update_trig.get_wifi_new_con == true && table_update_trig.get_wifi_ok == true){
		wifi_connect_to_lcd_hex(2);
	}
	
	
	wifi_status_to_lcd_hex(wifi_cfg.wif_conect_status);
	
	air_status_to_lcd_hex(to_app.air_status);
	heat_status_to_lcd_hex(to_app.relay_status.relay.heat);
	humi_status_lcd_hex(to_app.relay_status.relay.humi);
	aerathion_status_to_lcd_hex(to_app.relay_status.relay.breath);
	win_status_to_lcd_hex(to_app.relay_status.relay.wind);
	sensitive_staus_to_lcd_hex(to_app.relay_status.relay.sensitive);
	light_status_to_lcd_hex(to_app.relay_status.relay.light);
	sterilize_staus_to_lcd_hex(to_app.relay_status.relay.sterilize);
	dry_status_to_lcd_hex(to_app.relay_status.relay.dry);
	
	
	for(uint8_t i=0;i<4;i++){
			sprintf(buf,"%s-%02d",to_app.id,i+1);
		  write_txt_data(TXT_WRITE_FUN_CMD, sensor_show[i].sensor_id_addr,buf);
		  if(to_app.signle_senser[i].senser_status == SENSER_OK){
				sprintf(buf,"%2.1f",(float)to_app.signle_senser[i].single_temp/10);
				write_txt_data(TXT_WRITE_FUN_CMD, sensor_show[i].sigle_sensor_temp_addr,buf);
				sprintf(buf,"%2.1f",(float)to_app.signle_senser[i].signle_humi/10);
				write_txt_data(TXT_WRITE_FUN_CMD, sensor_show[i].sigle_sensor_humi_addr,buf);
			}else{
				write_txt_data(TXT_WRITE_FUN_CMD, sensor_show[i].sigle_sensor_temp_addr,"未安装");
				write_txt_data(TXT_WRITE_FUN_CMD, sensor_show[i].sigle_sensor_humi_addr,"未安装");
			}
 }
	static char* txt;static bool flag = 0;
  switch(work_way_status.work_status){
		case BAOZHONG_STA:
			   txt = "保种运行";flag = 1;
		    break;
		case BAOZHONG_END:
			   txt = "保种停止";flag = 0;
		    break;
		case GONGYU_STA:
			   txt = "共育运行";flag = 1;
		    break;
		case GONGYU_END:
			    txt = "共育停止";flag = 0;
		    break;
		case CUIQING_STA:
			  txt = "催青运行";flag = 1;
		    break;
		case CUIQING_END:
			   txt = "催青停止";flag = 0;
		    break;
	}
  work_mode_status_to_lcd_hex(flag);
  set_work_mode_to_lcd_txt(txt);
	
	sprintf(buf,"%s",secs_to_time(to_app.next_light.time_sta));
	light_start_to_lcd_txt(buf);
	sprintf(buf,"%s",secs_to_time(to_app.next_light.time_end));
	light_end_to_lcd_txt(buf);
	
	sprintf(buf,"%s",secs_to_time(to_app.next_sensitive.time_sta));
	sersitive_start_to_lcd_txt(buf);
	sprintf(buf,"%s",secs_to_time(to_app.next_sensitive.time_end));
	sersitive_time_to_lcd_txt(buf);
	
	sprintf(buf,"%s",secs_to_time(to_app.next_breath.time_sta));
	aeration_start_to_lcd_txt(buf);
	sprintf(buf,"%02d",to_app.next_breath.min_len);
	aeration_end_to_lcd_txt(buf);
	
	sprintf(buf,"%s",secs_to_time(to_app.next_sterilize.time_sta));
	sterilize_start_to_lcd_txt(buf);
	sprintf(buf,"%02d",to_app.next_sterilize.min_len);
	sterilize_time_to_lcd_txt(buf);
	
	/**********************************更新表数据到lcd*************************************/
	if(table_update_trig.mcu_to_lcd_trig == true){
		static uint8_t count = 0;
		count++;
		uint8_t buf[20]={0};
		table_update_trig.update_net_trig_send = true;
		
		sprintf((char*)buf,"%2.1f",(float)(all_table_data.set_sensor[0].temp_center)/10);
		temp_center_to_lcd_txt((char*)buf);
		temp_home_center_to_lcd_txt((char*)buf);
		
		sprintf((char*)buf,"%2.1f",(float)(all_table_data.set_sensor[0].humi_center)/10);
		humi_center_to_lcd_txt((char*)buf);
		humi_home_center_to_lcd_txt((char*)buf);
		
		sprintf((char*)buf,"%2.1f",(float)(all_table_data.set_sensor[0].temp_diff)/10);
		temp_diff_to_lcd_txt((char*)buf);
		sprintf((char*)buf,"%2.1f",(float)(all_table_data.set_sensor[0].humi_diff)/10);
		humi_diff_to_lcd_txt((char*)buf);
		
		sprintf((char*)buf,"%2.1f",(float)(all_table_data.set_wind_temp_diff)/10);
		uniform_return_diffrence_to_lcd_txt((char*)buf);
		
		for(uint8_t i=0;i<8;i++){
			write_txt_data(TXT_WRITE_FUN_CMD, breath_show[i].time_start_addr,secs_to_time(all_table_data.set_breath[i].time_sta));
			sprintf((char*)buf,"%02d",all_table_data.set_breath[i].min_len);
			write_txt_data(TXT_WRITE_FUN_CMD, breath_show[i].time_end_addr,(char*)buf);
		}
		
		for(uint8_t i=0;i<3;i++){
			write_txt_data(TXT_WRITE_FUN_CMD, sensitive_show[i].time_start_addr,secs_to_time(all_table_data.set_sensitive[i].time_sta));
			write_txt_data(TXT_WRITE_FUN_CMD, sensitive_show[i].time_end_addr,secs_to_time(all_table_data.set_sensitive[i].time_end));
		}
		
		for(uint8_t i=0;i<3;i++){
			write_txt_data(TXT_WRITE_FUN_CMD, light_show[i].time_start_addr,secs_to_time(all_table_data.set_light[i].time_sta));
			write_txt_data(TXT_WRITE_FUN_CMD, light_show[i].time_end_addr,secs_to_time(all_table_data.set_light[i].time_end));
		}
		
		for(uint8_t i=0;i<4;i++){
			write_txt_data(TXT_WRITE_FUN_CMD, sterilize_show[i].time_start_addr,secs_to_time(all_table_data.set_sterilize[i].time_sta));
			sprintf((char*)buf,"%02d",all_table_data.set_sterilize[i].min_len);
			write_txt_data(TXT_WRITE_FUN_CMD, sterilize_show[i].time_end_addr,(char*)buf);
		}
		
		for(uint8_t i=0;i<4;i++){
			sprintf((char*)buf,"%s-%02d",to_app.id,i+1);
			write_txt_data(TXT_WRITE_FUN_CMD, sensor_show[i].sensor_id_addr,(char*)buf);
			sprintf((char*)buf,"%2.1f",(float)to_app.signle_senser[i].single_temp/10);
			write_txt_data(TXT_WRITE_FUN_CMD, sensor_show[i].sigle_sensor_temp_addr,(char*)buf);
			sprintf((char*)buf,"%2.1f",(float)to_app.signle_senser[i].signle_humi/10);
			write_txt_data(TXT_WRITE_FUN_CMD, sensor_show[i].sigle_sensor_humi_addr,(char*)buf);
			sprintf((char*)buf,"%2.1f",(float)all_table_data.set_senser_diff[i].temp_adjust/10);
			write_txt_data(TXT_WRITE_FUN_CMD, sensor_show[i].sigle_sensor_adjust_temp_addr ,(char*)buf);
			sprintf((char*)buf,"%2.1f",(float)all_table_data.set_senser_diff[i].humi_adjust /10);
			write_txt_data(TXT_WRITE_FUN_CMD, sensor_show[i].sigle_sensor_adjust_humi_addr ,(char*)buf);
		}
		
		
		
		if(count >= 2)
			count = 0;
		if(count == 0){
			table_data= all_table_data;
		  table_data.crc = CRC16_2_modbus((uint8_t*)&table_data,sizeof(UPDATE_ALL_TABLE_TYPEDEF)-2);
		  flash_write((uint16_t*)&table_data,sizeof(UPDATE_ALL_TABLE_TYPEDEF));
			if(table_update_trig.mcu_to_lcd_trig == true)table_update_trig.mcu_to_lcd_trig = false;
		}
	}
	
	/**********************************lcd表数据更新表到本地*************************************/

	if(table_update_trig.lcd_to_mcu_trig == true){
		static uint8_t count=0;
		count++;
		p = temp_center_from_lcd_txt();
		if(p != NULL){
			all_table_data.set_sensor[0].temp_center = atof(p)*10;
			sprintf((char*)buf,"%2.1f",(float)(all_table_data.set_sensor[0].temp_center)/10);
			temp_home_center_to_lcd_txt((char*)buf);
		}
		p = temp_diff_from_lcd_txt();
		if(p != NULL){
			all_table_data.set_sensor[0].temp_diff = atof(p)*10;
		}
		
		p = humi_center_from_lcd_txt();
		if(p != NULL){
			all_table_data.set_sensor[0].humi_center = atof(p)*10;
			sprintf((char*)buf,"%2.1f",(float)(all_table_data.set_sensor[0].humi_center)/10);
			humi_home_center_to_lcd_txt((char*)buf);
		}
		p = humi_diff_from_lcd_txt();
		if(p != NULL){
			all_table_data.set_sensor[0].humi_diff = atof(p)*10;
		}
		
		p = uniform_return_diffrence_from_lcd_txt();
		if(p != NULL){
			all_table_data.set_wind_temp_diff = atof(p)*10;
		}
		
		
		for(uint8_t i = 0;i<8;i++){
			
			p = read_txt_data(TXT_GET_FUN_CMD, TXT_READ_FUN_CMD, breath_show[i].time_start_addr);
			if(p != NULL){
				 all_table_data.set_breath[i].time_sta = time_to_secs(p);
			}
			p = read_txt_data(TXT_GET_FUN_CMD, TXT_READ_FUN_CMD, breath_show[i].time_end_addr);
			if(p != NULL){
				 all_table_data.set_breath[i].min_len = atoi(p);
			}
		}
		
		for(uint8_t i = 0;i<3;i++){
			
			p = read_txt_data(TXT_GET_FUN_CMD, TXT_READ_FUN_CMD, sensitive_show[i].time_start_addr);
			if(p != NULL){
				 all_table_data.set_sensitive[i].time_sta = time_to_secs(p);
			}
			p = read_txt_data(TXT_GET_FUN_CMD, TXT_READ_FUN_CMD, sensitive_show[i].time_end_addr);
			if(p != NULL){
				 all_table_data.set_sensitive[i].time_end = time_to_secs(p);
			}
		}
		
		for(uint8_t i = 0;i<3;i++){
			
			p = read_txt_data(TXT_GET_FUN_CMD, TXT_READ_FUN_CMD, light_show[i].time_start_addr);
			if(p != NULL){
				 all_table_data.set_light[i].time_sta = time_to_secs(p);
			}
			p = read_txt_data(TXT_GET_FUN_CMD, TXT_READ_FUN_CMD, light_show[i].time_end_addr);
			if(p != NULL){
				 all_table_data.set_light[i].time_end = time_to_secs(p);
			}
		}
		
		
		for(uint8_t i = 0;i<4;i++){
			
			p = read_txt_data(TXT_GET_FUN_CMD, TXT_READ_FUN_CMD, sterilize_show[i].time_start_addr);
			if(p != NULL){
				 all_table_data.set_sterilize[i].time_sta = time_to_secs(p);
			}
			p = read_txt_data(TXT_GET_FUN_CMD, TXT_READ_FUN_CMD, sterilize_show[i].time_end_addr);
			if(p != NULL){
				 all_table_data.set_sterilize[i].min_len = atoi(p);
			}
		}
		
		
		for(uint8_t i = 0;i<4;i++){
			p = read_txt_data(TXT_GET_FUN_CMD, TXT_READ_FUN_CMD, sensor_show[i].sigle_sensor_adjust_temp_addr);
			if(p != NULL){
				all_table_data.set_senser_diff[i].temp_adjust = atof(p)*10;
			}
			p = read_txt_data(TXT_GET_FUN_CMD, TXT_READ_FUN_CMD, sensor_show[i].sigle_sensor_adjust_humi_addr);
			if(p != NULL){
				all_table_data.set_senser_diff[i].humi_adjust = atof(p)*10;
			}
		}
		if(count >= 2){
			count = 0;
			table_data= all_table_data;
		  table_data.crc = CRC16_2_modbus((uint8_t*)&table_data,sizeof(UPDATE_ALL_TABLE_TYPEDEF)-2);
		  flash_write((uint16_t*)&table_data,sizeof(UPDATE_ALL_TABLE_TYPEDEF));
			table_update_trig.write_flash = true;
			table_update_trig.mcu_to_net_trig = true;
			table_update_trig.mcu_to_pc_trig = true;
			table_update_trig.lcd_to_mcu_trig = false;
		}
	}
	
}

void set_default_parameters(char *id)
{
		const uint8_t parament[165] = {0xA0,0x4C,0x53,0x53,0x46,0x32,0x31,0x30,0x36,0x30,0x31,0x30,0x30,0x33,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x30,0x2A,0x00,0x00,0x05,0x60,0x54,0x00,0x00,0x05,0x90,0x7E,0x00,0x00,0x05,0xC0,0xA8,0x00,0x00,0x05,0xF0,0xD2,0x00,0x00,0x05,0x20,0xFD,0x00,0x00,0x05,0x50,0x27,0x01,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFA,0x00,0x14,0x00,0x52,0x03,0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0xA1,0x02,0x3D};
		UPDATE_ALL_TABLE_TYPEDEF table_data;
		memcpy(&table_data,parament,sizeof(UPDATE_ALL_TABLE_TYPEDEF));
		memcpy(&table_data.id,id,13);
		//table_update_trig.change_id = true;
		memcpy(to_app.id,table_data.id,sizeof(all_table_data.id));
			
		table_data.crc = CRC16_2_modbus((uint8_t*)&table_data,sizeof(UPDATE_ALL_TABLE_TYPEDEF)-2);
		flash_write((uint16_t*)&table_data,sizeof(UPDATE_ALL_TABLE_TYPEDEF));
		all_table_data = table_data;
		table_update_trig.mcu_to_lcd_trig = true;
}

void write_id_data(void)
{
	const uint8_t send1[] = {0xEE,0xB1,0x11,0x00,0x0D,0x00,0x01,0x10,0x00,0x01,0xFF,0xFC,0xFF,0xFF};
	const uint8_t send2[] = {0xEE,0xB1,0x11,0x00,0x0D,0x00,0x01,0x10,0x00,0x00,0xFF,0xFC,0xFF,0xFF};
	UNUSED(huart2);
	HAL_UART_Transmit(&huart2,(uint8_t*)&send1,sizeof(send1),0xffff);
	vTaskDelay(100);
	save_devId_to_lcd_txt(all_table_data.id);
	vTaskDelay(100);
	HAL_UART_Transmit(&huart2,(uint8_t*)&send2,sizeof(send1),0xffff);
	
}

void write_id_data1(void)
{
	const uint8_t send1[] = {0xEE,0xB1,0x10,0x00,0x14,0x00,0x03,0x01,0xFF,0xFC,0xFF,0xFF};
	const uint8_t send2[] = {0xEE,0xB1,0x10,0x00,0x14,0x00,0x03,0x00,0xFF,0xFC,0xFF,0xFF};
	UNUSED(huart2);
	change_id_to_lcd_txt(all_table_data.id);
	HAL_UART_Transmit(&huart2,(uint8_t*)&send1,sizeof(send1),0xffff);
	vTaskDelay(1000);
	vTaskDelay(1000);
	HAL_UART_Transmit(&huart2,(uint8_t*)&send2,sizeof(send1),0xffff);
}


