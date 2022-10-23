#ifndef __GET_TEMP_H
#define __GET_TEMP_H
#include "config.h"
#include <stdio.h>
#include <stdlib.h>
extern LOCAL_TEMP_HUMI_TypeDef sensor_adj;
extern TEMP_HUMI_TypeDef local_temp_humi_data;
extern LOCAL_TEMP_HUMI_TypeDef local_temp_humi_to_lcd;
TEMP_HUMI_TypeDef* read_temp_humi(uint8_t sensor_id);
uint16_t swap_16(uint16_t data);
uint32_t swap_32(uint32_t val);
#endif
