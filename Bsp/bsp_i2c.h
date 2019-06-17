#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "stm32f1xx_hal.h"
#include "i2c.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
#define I2C_OWN_ADDRESS                            0x0A              // stm32����I2C��ַ
#define I2C_SPEEDCLOCK                             200000            // I2Cͨ������(���Ϊ400K)
#define I2C_DUTYCYCLE                              I2C_DUTYCYCLE_2   // I2Cռ�ձ�ģʽ��1/2 

#define MPU6050_I2C_RCC_CLK_ENABLE()                __HAL_RCC_I2C1_CLK_ENABLE()
#define MPU6050_I2C_RCC_CLK_DISABLE()               __HAL_RCC_I2C1_CLK_DISABLE()

#define MPU6050_I2C_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define MPU6050_I2C_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOB_CLK_DISABLE()   
#define MPU6050_I2C_GPIO_PORT                       GPIOB   
#define MPU6050_I2C_SCL_PIN                         GPIO_PIN_6
#define MPU6050_I2C_SDA_PIN                         GPIO_PIN_7

/* ��չ���� ------------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c_mpu6050;

/* �������� ------------------------------------------------------------------*/
static void BSP_I2C_MPU6050_Error (void);
void BSP_I2C_MPU6050_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value);
HAL_StatusTypeDef BSP_I2C_MPU6050_WriteBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
uint8_t BSP_I2C_MPU6050_ReadData(uint16_t Addr, uint8_t Reg);
HAL_StatusTypeDef BSP_I2C_MPU6050_ReadBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
HAL_StatusTypeDef BSP_I2C_MPU6050_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);

#endif
