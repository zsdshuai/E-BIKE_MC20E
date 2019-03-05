#include "kx023.h"
#include "stdbool.h"

extern I2C_HandleTypeDef hi2c1;
#define SHAKE_BUF_LEN 2

uint8_t zt_gsensor_curr_shake_value_array[MAX_SHAKE_NUM];
uint8_t flag_shake;
uint8_t shake_buf[SHAKE_BUF_LEN] = {0};
uint8_t shake_buf_index = 0;

zt_acce_struct zt_acce = {0};

void kx023_read(uint8_t *x, uint8_t *y, uint8_t *z);

uint8_t abs_S8(uint8_t v)
{
	if(v<128) 
	{
		return v;
	}
	else
	{
		return 256-v;
	}
}

uint16_t zt_gsensor_get_curr_max_shake_value(uint16_t during_times)
{
	uint16_t i,max_value=0;

	for(i=0; i<during_times; i++)
	{
		if(zt_gsensor_curr_shake_value_array[i] > max_value)
		{
			max_value = zt_gsensor_curr_shake_value_array[i];
		}
	}
	return max_value;
}
/*****************************************************************************
 * FUNCTION
 * zt_gsensor_get_shake_num
 * DESCRIPTION
 * 获取一段时间gsensor震动值
 * PARAMETERS
 * uint16_t shake_value  检测的震动值
 * uint16_t during_times 最近时间内, 单位秒, 最大为60s
 * RETURNS
 *  uint16_t  返回大于shake_value的个数
 *****************************************************************************/
uint16_t zt_gsensor_get_shake_num(uint16_t shake_value, uint16_t during_times)
{
	uint16_t i,num=0;

	for(i=0; i<during_times; i++)
	{
		if(zt_gsensor_curr_shake_value_array[i] > shake_value)
		{
			num++;
		}
	}
	return num;
}

/*****************************************************************************
 * FUNCTION
 *  zt_gsensor_check_is_shake_sharp
 * DESCRIPTION
 *  检测是否剧烈运动
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  bool
 *****************************************************************************/
bool zt_gsensor_check_is_shake_sharp(void)
{
	uint16_t num;
	
	num = zt_gsensor_get_shake_num(10, 5);

	if(num>=2)
		return true;
	else
		return false;
}

/*****************************************************************************
 * FUNCTION
 *  zt_gsensor_check_is_moving
 * DESCRIPTION
 *  检测是否移动
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  bool
 *****************************************************************************/
bool zt_gsensor_check_is_moving(void)
{
	uint16_t num;
	num = zt_gsensor_get_shake_num(5, 15);

	if(num>=10)
		return true;
	else
		return false;
}
/*****************************************************************************
 * FUNCTION
 *  zt_gsensor_check_is_motionless
 * DESCRIPTION
 *   检测是否静止
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  bool
 *****************************************************************************/
bool zt_gsensor_check_is_motionless(void)
{
	uint16_t num;

	num = zt_gsensor_get_shake_num(5, MAX_SHAKE_NUM);

	if(num < 10)
		return true;
	else
		return false;
}
/*****************************************************************************
 * FUNCTION
 *  zt_gsensor_get_shake_change
 * DESCRIPTION
 *   获取 震动数据改变值
 * PARAMETERS
 *  void  
 *  
 * RETURNS
 *  uint8_t
 *****************************************************************************/
uint8_t zt_gsensor_get_shake_change(void)
{ 
	uint8_t x=0,y=0,z=0;
	uint8_t a=0,b=0,c=0;
	uint8_t change = 0;
	uint8_t average_change = 0;
	uint8_t i;
	uint16_t sum = 0;

	kx023_read(&x, &y, &z);

	x &= 0x3f;
	y &= 0x3f;
	z &= 0x3f;

	if(x>32) x-=64;
	if(y>32) y-=64;
	if(z>32) z-=64;
	
	a=x;b=y;c=z;
	
	a-=zt_acce.x;
	b-=zt_acce.y;
	c-=zt_acce.z;
	
	change = abs_S8(a) + abs_S8(b) + abs_S8(c);
	
	zt_acce.x = x;
	zt_acce.y = y;
	zt_acce.z = z;  

	//进行数据平均
	shake_buf_index++;
	shake_buf_index %= SHAKE_BUF_LEN;
	shake_buf[shake_buf_index] = change;

	for(i = 0; i< SHAKE_BUF_LEN; i++)
		sum += shake_buf[i];
	
	average_change = sum/SHAKE_BUF_LEN;

	return average_change;
}


/*一秒钟获取一次震动数据到buf池*/
void collect_shake_data(void)
{			
	uint16_t index;

	for(index=MAX_SHAKE_NUM-1;index>=1;index--)
	{
		zt_gsensor_curr_shake_value_array[index]=zt_gsensor_curr_shake_value_array[index-1];
	}
	zt_gsensor_curr_shake_value_array[0] = zt_gsensor_get_shake_change();

}
void kx023_init(void)
{
	 uint8_t val;
	 //val = 0x40;
   val = 0x00;		//配置采集8bit
	 HAL_I2C_Mem_Write(&hi2c1, kx023_write_address, CNTL1, I2C_MEMADD_SIZE_8BIT, &val, 1, 0x10);
   //val = 0x00;
	 //HAL_I2C_Mem_Write(&hi2c1, kx023_write_address, BUF_CNTL2, I2C_MEMADD_SIZE_8BIT, &val, 1, 0x10);
	 val = 0x02;
   HAL_I2C_Mem_Write(&hi2c1, kx023_write_address, ODCNTL, I2C_MEMADD_SIZE_8BIT, &val, 1, 0x10);
   //val = 0xC0;
	 val = 0x80;
   HAL_I2C_Mem_Write(&hi2c1, kx023_write_address, CNTL1, I2C_MEMADD_SIZE_8BIT, &val, 1, 0x10);
}
void kx023_read(uint8_t *x, uint8_t *y, uint8_t *z)
{
	  uint8_t x_h, y_h, z_h;
		//HAL_I2C_Mem_Read(&hi2c1, kx023_read_address, XOUT_L, I2C_MEMADD_SIZE_8BIT, &x_l, 1, 0x10);
    HAL_I2C_Mem_Read(&hi2c1, kx023_read_address, XOUT_H, I2C_MEMADD_SIZE_8BIT, &x_h, 1, 0x10);
	  //HAL_I2C_Mem_Read(&hi2c1, kx023_read_address, YOUT_L, I2C_MEMADD_SIZE_8BIT, &y_l, 1, 0x10);
	  HAL_I2C_Mem_Read(&hi2c1, kx023_read_address, YOUT_H, I2C_MEMADD_SIZE_8BIT, &y_h, 1, 0x10);
	  //HAL_I2C_Mem_Read(&hi2c1, kx023_read_address, ZOUT_L, I2C_MEMADD_SIZE_8BIT, &z_l, 1, 0x10);
	  HAL_I2C_Mem_Read(&hi2c1, kx023_read_address, ZOUT_H, I2C_MEMADD_SIZE_8BIT, &z_h, 1, 0x10);
	  *x = x_h;//(x_h<<8) + x_l;
	  *y = y_h;//(y_h<<8) + y_l;
	  *z = z_h;//(z_h<<8) + z_l;
	  //printf ("x = %d, y = %d, z = %d \r\n", x, y, z);  
}



 
