#include "flash.h"
#include "stmflash.h"
#include "Stm32f0xx_hal_i2c.h"

#define ADDR_W 0xA0
#define ADDR_R 0xA1
#define PAGE_SIZE  256    //字节  //每个地址写入为的长度为4个字节，一页为1K字节

__IO uint16_t  sEEAddress = 0;   
__IO uint32_t  sEETimeout = sEE_LONG_TIMEOUT;
__IO uint16_t  sEEDataNum;


extern I2C_HandleTypeDef hi2c1;
#if 1
void write_flash(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumByteToWrite)
{
  uint16_t NumOfPage = 0, NumOfSingle = 0, count = 0;
  uint16_t Addr = 0;
  
  Addr = WriteAddr % sEE_PAGESIZE;
  count = sEE_PAGESIZE - Addr;
  NumOfPage =  NumByteToWrite / sEE_PAGESIZE;
  NumOfSingle = NumByteToWrite % sEE_PAGESIZE;
  
  /*!< If WriteAddr is sEE_PAGESIZE aligned  */
  if(Addr == 0) 
  {
    /*!< If NumByteToWrite < sEE_PAGESIZE */
    if(NumOfPage == 0) 
    {
      /* Store the number of data to be written */
      sEEDataNum = NumOfSingle;
      /* Start writing data */
      HAL_I2C_Mem_Write(&hi2c1, ADDR_W, WriteAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, sEEDataNum, 0x10);
      HAL_Delay(5);
    }
    /*!< If NumByteToWrite > sEE_PAGESIZE */
    else  
    {
      while(NumOfPage--)
      {
        /* Store the number of data to be written */
        sEEDataNum = sEE_PAGESIZE;        
        HAL_I2C_Mem_Write(&hi2c1, ADDR_W, WriteAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, sEEDataNum, 0x10);
        HAL_Delay(5);
        WriteAddr +=  sEE_PAGESIZE;
        pBuffer += sEE_PAGESIZE;
      }
      
      if(NumOfSingle!=0)
      {
        /* Store the number of data to be written */
        sEEDataNum = NumOfSingle;          
        HAL_I2C_Mem_Write(&hi2c1, ADDR_W, WriteAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, sEEDataNum, 0x10);
        HAL_Delay(5);
      }
    }
  }
  /*!< If WriteAddr is not sEE_PAGESIZE aligned  */
  else 
  {
    /*!< If NumByteToWrite < sEE_PAGESIZE */
    if(NumOfPage== 0) 
    {
      /*!< If the number of data to be written is more than the remaining space 
      in the current page: */
      if (NumByteToWrite > count)
      {
        /* Store the number of data to be written */
        sEEDataNum = count;        
        /*!< Write the data conained in same page */
        HAL_I2C_Mem_Write(&hi2c1, ADDR_W, WriteAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, sEEDataNum, 0x10);
        HAL_Delay(5);
        
        /* Store the number of data to be written */
        sEEDataNum = (NumByteToWrite - count);          
        /*!< Write the remaining data in the following page */
        HAL_I2C_Mem_Write(&hi2c1, ADDR_W, (WriteAddr + count), I2C_MEMADD_SIZE_8BIT, (pBuffer + count), sEEDataNum, 0x10);
		
         HAL_Delay(5);
      }      
      else      
      {
        /* Store the number of data to be written */
        sEEDataNum = NumOfSingle;         
        HAL_I2C_Mem_Write(&hi2c1, ADDR_W, WriteAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, sEEDataNum, 0x10);
        HAL_Delay(5);
      }     
    }
    /*!< If NumByteToWrite > sEE_PAGESIZE */
    else
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / sEE_PAGESIZE;
      NumOfSingle = NumByteToWrite % sEE_PAGESIZE;
      
      if(count != 0)
      {  
        /* Store the number of data to be written */
        sEEDataNum = count;         
        HAL_I2C_Mem_Write(&hi2c1, ADDR_W, WriteAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, sEEDataNum, 0x10);
        HAL_Delay(5);
        WriteAddr += count;
        pBuffer += count;
      } 
      
      while(NumOfPage--)
      {
        /* Store the number of data to be written */
        sEEDataNum = sEE_PAGESIZE;          
        HAL_I2C_Mem_Write(&hi2c1, ADDR_W, WriteAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, sEEDataNum, 0x10);
        sEETimeout = sEE_LONG_TIMEOUT;
        HAL_Delay(5);
        WriteAddr +=  sEE_PAGESIZE;
        pBuffer += sEE_PAGESIZE;  
      }
      if(NumOfSingle != 0)
      {
        /* Store the number of data to be written */
        sEEDataNum = NumOfSingle;           
        HAL_I2C_Mem_Write(&hi2c1, ADDR_W, WriteAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, sEEDataNum, 0x10);
        HAL_Delay(5);
      }
    }
  }  
}

void read_flash(uint16_t ReadAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{
	uint16_t i;
	uint8_t ret;

//	for(i=0; i<NumByteToRead; i++)
	{
		ret = HAL_I2C_Mem_Read(&hi2c1, ADDR_R, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToRead, 0x10);
		HAL_Delay(1);
//		printf("\r\nread ret=%d\r\n",ret);
	}
}


#else
void write_flash(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumByteToWrite)
{
//	STMFLASH_Write(write_addr,buffer,size);
	uint16_t i;
	uint8_t ret;

	for(i=0; i<NumByteToWrite/16; i++)
	{
	 	ret = HAL_I2C_Mem_Write(&hi2c1, ADDR_W, WriteAddr+i*16, I2C_MEMADD_SIZE_8BIT, pBuffer+i*16, 16, 0x10);
		HAL_Delay(1);
		printf("\r\nwrite ret=%d\r\n",ret);
	}

}
void read_flash(uint32_t read_addr, uint8_t* buffer, uint16_t size)
{
	uint16_t i;
	uint8_t ret;

	for(i=0; i<size/16; i++)
	{
		ret = HAL_I2C_Mem_Read(&hi2c1, ADDR_R, read_addr+i*16, I2C_MEMADD_SIZE_8BIT, buffer+i*16, 16, 0x10);
		HAL_Delay(1);
		printf("\r\nread ret=%d\r\n",ret);
	}
//	STMFLASH_Read(read_addr, buffer,size);   	
}
#endif

#if 0
//FLASH写入数据
void write_flash(uint32_t read_addr, uint16_t *buffer, uint16_t size)
{
	  uint32_t i;
    HAL_FLASH_Unlock();
		//擦除FLASH
    //初始化FLASH_EraseInitTypeDef
    FLASH_EraseInitTypeDef f;
    f.TypeErase = FLASH_TYPEERASE_PAGES;
    f.PageAddress = write_addr;
	  if (size > 256)
			f.NbPages = 2;
		else if (size > 512)
			f.NbPages = 3;
		else
			f.NbPages = 1;
    //设置PageError
    uint32_t PageError = 0;
    //调用擦除函数
    HAL_FLASHEx_Erase(&f, &PageError);
    
    //对FLASH写
	for (i = 0; i < size; i++) 
  	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, write_addr, buffer[i]);
  			write_addr += 4;
	}
	HAL_FLASH_Lock();
}
//FLASH读取数据
void read_flash(uint32_t read_addr, uint16_t* buffer, uint16_t size)
{
	 uint16_t i;
	 
	 for (i = 0; i < size; i++) 
  	{
		buffer[i] = *(__IO uint32_t*)(read_addr);
		printf("addr:0x%x, data:0x%x\r\n", read_addr, buffer[i]);
		read_addr += 4;
	}
}
#endif



