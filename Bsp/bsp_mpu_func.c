#include "includes.h"


uint8_t i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
	HAL_Delay(1);
	if(HAL_I2C_Mem_Write(&hi2c2,slave_addr,reg_addr,I2C_MEMADD_SIZE_8BIT,(uint8_t*)data,length,0xff)!=HAL_OK)
	{
		printf("\r\nHAL_I2C_Mem_Write Error\r\n");
		return -1;
	}
	return 0;
}

uint8_t i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{ 
	HAL_Delay(1);
	if(HAL_I2C_Mem_Read(&hi2c2,slave_addr,reg_addr,I2C_MEMADD_SIZE_8BIT,(uint8_t*)data,length,0xff)!=HAL_OK)
	{
		printf("\r\nHAL_I2C_Mem_Read Error\r\n");
		return -1;
	}
	return 0;
}
