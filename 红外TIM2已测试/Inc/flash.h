//================================================================================
//STM32F207         
//================================================================================
//FLASH_S.C/H       Æ¬ÄÚFLASH¶ÁÐ´º¯Êý                          
//================================================================================
#ifndef __FLASH_S_H
#define __FLASH_S_H
#include "config.h"
//================================================================================
HAL_StatusTypeDef flash_write(uint16_t* p,uint16_t len);
void flash_read(uint16_t* p,uint16_t len);
HAL_StatusTypeDef flash__write(uint16_t* p,uint16_t len);
void flash__read(uint16_t* p,uint16_t len);

void readFlashTest(uint16_t* p,uint16_t len,uint8_t mode);
void writeFlashTest(uint16_t* p,uint16_t len,uint8_t mode);
#endif


