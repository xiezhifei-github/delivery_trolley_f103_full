#ifndef BSP_FILTER_H
#define BSP_FILTER_H

#include "stm32f1xx_hal.h"

#include "math.h"

float bsp_lpf_fitler(float current, float index);

#endif
