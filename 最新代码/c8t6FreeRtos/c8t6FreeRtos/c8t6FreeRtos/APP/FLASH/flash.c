
//================================================================================
#include "flash.h"
//================================================================================

#define BASE_ADDR  0x0800C800
#define BASE__ADDR 0x0800CC00

const uint32_t 	addr 				=   0x0800C800;
const uint32_t 	addr_r 			= 	0x0800CC00;

#define HEAT_ADDR	 0x0800D000
#define COLD_ADDR	 0x0800D400
#define CLOSE_ADDR 0x0800D800



const uint32_t 	addr_HEAT  	=		0x0800D000;
const uint32_t  addr_COLD  	=		0x0800D400;
const uint32_t  addr_CLOSE  =		0x0800D800;

HAL_StatusTypeDef flash_write(uint16_t* p,uint16_t len)
{
		//1、解锁FLASH
  HAL_FLASH_Unlock();
	static HAL_StatusTypeDef halRet;
	
	//2、擦除FLASH
	//初始化FLASH_EraseInitTypeDef
	FLASH_EraseInitTypeDef f;
	f.TypeErase = FLASH_TYPEERASE_PAGES;
	f.PageAddress = addr;
	f.NbPages = 1;
	
//	flash_store.work_way_status.reasure_count++;
//	work_way_status.reasure_count = flash_store.work_way_status.reasure_count;
	
	//设置PageError
	uint32_t PageError = 0;
	__disable_irq();
	//调用擦除函数
	halRet = HAL_FLASHEx_Erase(&f, &PageError);

	//3、对FLASH烧写
	
	if( halRet == HAL_ERROR )
				return HAL_ERROR;
	
	for( uint16_t i = 0 ; i < len ; i++ )
	{
		HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, addr + 2 * i, p[i]);
	}
	
	//4、锁住FLASH
  HAL_FLASH_Lock();
	__enable_irq ();
	
	return HAL_OK;
}

HAL_StatusTypeDef flash__write(uint16_t* p,uint16_t len)
{
		//1、解锁FLASH
  HAL_FLASH_Unlock();
	
	static HAL_StatusTypeDef halRet;
	//2、擦除FLASH
	//初始化FLASH_EraseInitTypeDef
	FLASH_EraseInitTypeDef f;
	f.TypeErase = FLASH_TYPEERASE_PAGES;
	f.PageAddress = addr_r;
	f.NbPages = 1;
	
//	flash_store.work_way_status.reasure_count++;
//	work_way_status.reasure_count = flash_store.work_way_status.reasure_count;
	
	//设置PageError
	uint32_t PageError = 0;
	__disable_irq();
	//调用擦除函数
	halRet = HAL_FLASHEx_Erase(&f, &PageError);
	if( halRet == HAL_ERROR )
					return HAL_ERROR;
	//3、对FLASH烧写
	
	
	for( uint16_t i = 0 ; i < len ; i++ )
	{
		HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, addr_r + 2 * i, p[i]);
	}
	
	//4、锁住FLASH
  HAL_FLASH_Lock();
	__enable_irq ();
	
	return HAL_OK;
}

void flash_read(uint16_t* p,uint16_t len)
{
	static volatile uint32_t addr = BASE_ADDR;
	addr = BASE_ADDR;
	while(len--){
		 *(p++)=*((__IO uint16_t*)addr);
		 addr = addr + 2;
	}
}

void flash__read(uint16_t* p,uint16_t len)
{
	static volatile uint32_t addr = BASE__ADDR;
	addr = BASE__ADDR;
	while(len--){
		 *(p++)=*((__IO uint16_t*)addr);
		 addr = addr + 2;
	}
}




//FLASH写入数据测试
HAL_StatusTypeDef writeFlashTest(uint16_t* p,uint16_t len,uint8_t mode)
{
	if( mode >= 3 )
		return HAL_ERROR;
	//1、解锁FLASH
  HAL_FLASH_Unlock();
	
	//2、擦除FLASH
	//初始化FLASH_EraseInitTypeDef
	
	FLASH_EraseInitTypeDef f;
	f.TypeErase = FLASH_TYPEERASE_PAGES;
	switch(mode){
		case AIR_HEAT:
				f.PageAddress = addr_HEAT;
			break;
		case AIR_COLD:
			f.PageAddress = addr_COLD;
			break;
		case AIR_CLOSE:
			f.PageAddress = addr_CLOSE;
			break;
		default:
			return	HAL_ERROR;
	}
	
	f.NbPages = 1;
	
	
	//设置PageError
	uint32_t PageError = 0;
	__disable_irq();
	//调用擦除函数
	HAL_FLASHEx_Erase(&f, &PageError);

	//3、对FLASH烧写
	
	for( uint16_t i = 0 ; i < len ; i++ )
	{
		HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, f.PageAddress + 2 * i, p[i]);
	}
	//4、锁住FLASH
  HAL_FLASH_Lock();
	__enable_irq ();
	return HAL_OK;
}

//FLASH读取数据测试

void readFlashTest(uint16_t* p,uint16_t len,uint8_t mode)
{
	if( mode > 3 )
		return ;
	static volatile uint32_t addr = BASE__ADDR;
	switch(mode){
		case AIR_HEAT:
				addr = HEAT_ADDR;
			break;
		case AIR_COLD:
			addr = COLD_ADDR;
			break;
		case AIR_CLOSE:
			addr = CLOSE_ADDR;
			break;
		default:
			return ;
	}
	
	while(len--){
		 *(p++)=*((__IO uint16_t*)addr);
		 addr = addr + 2;
	}

}

