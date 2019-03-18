#ifndef BSP_MOTO_DRIVE_H
#define BSP_MOTO_DRIVE_H

#include "stm32f1xx_hal.h"

#include "gpio.h"
//#include "dma.h"
#include "i2c.h"
#include "tim.h"

#include "bsp_control.h"

typedef int DirectionTypeDef;
#define moto_direction_forward 0
#define moto_direction_back 1

typedef int MotoIDTypeDef;
#define moto_ID_1 0
#define moto_ID_2 1

//#define BSP_Encoder_GetValue __HAL_TIM_GetCounter
#define BSP_PWM_SetValue __HAL_TIM_SET_COMPARE

uint32_t BSP_Moto_GetValue(MotoIDTypeDef moto_id);
void BSP_Moto_SetSpeed(MotoIDTypeDef moto_id,DirectionTypeDef direction, uint32_t value);
void BSP_Moto_SpeedControlOutput(moto_pid *pid, uint8_t cnt);
void BSP_Moto_Control(double value1, double value2);

#endif
