#ifndef __KX023_H
#define __KX023_H	 
#include "stm32f0xx_hal.h"
#include "stdbool.h"

#define kx023_write_address 0x3C
#define kx023_read_address  0x3D

//function Registers address
#define   BUF_CNTL2            0x3B
#define 	CNTL1                0x18
#define   ODCNTL               0x1B
#define 	XHP_L                0x00
#define		XHP_H							   0x01
#define   YHP_L                0x02
#define		YHP_H							   0x03
#define		ZHP_L							   0x04
#define		ZHP_H							 	 0x05
#define		XOUT_L							 0x06
#define		XOUT_H							 0x07
#define		YOUT_L							 0x08
#define		YOUT_H							 0x09
#define		ZOUT_L							 0x0A
#define		ZOUT_H							 0x0B

typedef struct{
	uint8_t x;
	uint8_t y;
	uint8_t z;
}zt_acce_struct;

#define MAX_SHAKE_NUM 300	//60

extern uint8_t flag_shake;

void kx023_init(void);	
void collect_shake_data(void);
//bool zt_gsensor_check_is_moving(void);
bool zt_gsensor_check_is_shake_sharp(void);
void collect_shake_data(void);

//void kx023_read(void);

#endif

