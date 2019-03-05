#ifndef __FLASH_H
#define __FLASH_H

#include "stm32f0xx_hal.h"
//exterm uint32_t writeflashdata;
#define 	STM32_FLASH_ADDRESS		0x0800f000 

static FLASH_EraseInitTypeDef EraseInitStruct;

//void writeFlash(uint32_t addr, uint32_t data);
//void readflash(uint32_t addr);
void write_flash(uint32_t write_addr, uint16_t *buffer, uint16_t size);
void read_flash(uint32_t read_addr, uint16_t* buffer, uint16_t size);


#endif
