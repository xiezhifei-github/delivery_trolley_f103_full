#ifndef BSP_LOCK_H
#define BSP_LOCK_H

#include "includes.h"

#define LOCK_OPEN     0
#define LOCK_CLOSE    1

#define BSP_Lock_Open()    HAL_GPIO_WritePin(LOCK_GPIO_Port,LOCK_Pin,GPIO_PIN_RESET)
#define BSP_Lock_Close()   HAL_GPIO_WritePin(LOCK_GPIO_Port,LOCK_Pin,GPIO_PIN_SET);

void BSP_Lock_Control(void);


#endif
