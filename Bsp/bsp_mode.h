#ifndef BSP_MODE_H
#define BSP_MODE_H

#include "includes.h"

typedef uint8_t control_mode;
#define BSP_MODE_Switch()        mode = !mode
	
#define control_mode_ps2         0x00
#define control_mode_pc          0x01

void BSP_Mode_Control(void);

#endif
