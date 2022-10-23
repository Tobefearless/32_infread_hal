#include "get_temp.h"
#include "crc.h"
#include "string.h"
LOCAL_TEMP_HUMI_TypeDef sensor_adj = {0.0,0.0};
TEMP_HUMI_TypeDef local_temp_humi_data;
LOCAL_TEMP_HUMI_TypeDef local_temp_humi_to_lcd;


TEMP_HUMI_TypeDef* read_temp_humi(uint8_t sensor_id)
{
	static TEMP_HUMI_TypeDef data;
	static SHT30_WRITE_TypeDef sht30_write;
	static RSHT30_READ_TypeDef sht30_read;
	static uint16_t crc;
	sht30_write.id = sensor_id;
	sht30_write.cmd = READ_DATA;
	sht30_write.addr = READ_TEMP_ADDR;
	sht30_write.data_len = 0X0200;
	
	sht30_write.crc = CRC16_2_modbus((uint8_t*)&sht30_write,sizeof(SHT30_WRITE_TypeDef)-2);
	
	HAL_UART_Transmit(&huart1,(uint8_t*)&sht30_write,sizeof(SHT30_WRITE_TypeDef),0xffff);
	usart1_rcv.rcv_ok = false;
	vTaskDelay(20);
	if(usart1_rcv.rcv_ok){
		usart1_rcv.rcv_ok = false;
		memcpy((uint8_t*)&sht30_read,usart1_rcv.arry,sizeof(RSHT30_READ_TypeDef));
		
		sht30_read.crc = swap_16(sht30_read.crc);
		sht30_read.temp = swap_16(sht30_read.temp);
		sht30_read.humi = swap_16(sht30_read.humi);
		
		crc = CRC16_2_modbus((uint8_t *)usart1_rcv.arry,sizeof(RSHT30_READ_TypeDef)-2);
		crc = swap_16(crc);
		if(crc == sht30_read.crc && sht30_read.id == sensor_id){
			data.temp = sht30_read.temp;
			data.humi = sht30_read.humi;
			data.id = sht30_read.id;
			return &data;
		}
		
    memset(usart1_rcv.arry,0,sizeof(RSHT30_READ_TypeDef));
	}
  return NULL;

}


uint16_t swap_16(uint16_t data)
{
	data = ((data >>8) & 0x00ff) | ((data <<8)&0xff00);
	return data;
}

uint32_t swap_32(uint32_t val)
{
	val = (val<<24) | ((val<<8) & 0x00ff0000) |
		  ((val>>8) & 0x0000ff00) | (val>>24);
	return val;
}

