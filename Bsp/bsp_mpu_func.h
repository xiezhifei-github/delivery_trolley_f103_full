#ifndef BSP_MPU_FUNC_H
#define BSP_MPU_FUNC_H

#include "stm32f1xx_hal.h"

#include "i2c.h"

#include "bsp_i2c.h"
#include "bsp_mpu6050.h"

uint8_t i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
uint8_t i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif
