#include "includes.h"

/**********************************************
Function name:   i2c_write
Features:        i2c写函数
Parameter:       slave_addr---从机地址，用以区分从机
                 reg_addr---待写寄存器地址
								 length---待写数据长度
								 data---待写数据指针
Return value:    0
**********************************************/
uint8_t i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
	HAL_Delay(1);
	if(HAL_I2C_Mem_Write(&hi2c2,slave_addr,reg_addr,I2C_MEMADD_SIZE_8BIT,(uint8_t*)data,length,0xff)!=HAL_OK)
	{
		printf("\r\nHAL_I2C_Mem_Write Error\r\n");
		return 0;
	}
	return 0;
}

/**********************************************
Function name:   i2c_read
Features:        i2c读函数
Parameter:       slave_addr---从机地址，用以区分从机
                 reg_addr---待读取寄存器地址
								 length---待读取数据长度
								 data---数据缓存指针
Return value:    0
**********************************************/
uint8_t i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{ 
	HAL_Delay(1);
	if(HAL_I2C_Mem_Read(&hi2c2,slave_addr,reg_addr,I2C_MEMADD_SIZE_8BIT,(uint8_t*)data,length,0xff)!=HAL_OK)
	{
		printf("\r\nHAL_I2C_Mem_Read Error\r\n");
		return 0;
	}
	return 0;
}
