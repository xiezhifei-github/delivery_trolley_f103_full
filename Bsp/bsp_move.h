#ifndef BSP_MOVE_H
#define BSP_MOVE_H

#include "includes.h"

#define MOVE_FORWARD(i) 			set_spd[0]=i;set_spd[1]=i;
#define MOVE_BACK(i) 					set_spd[0]=-i;set_spd[1]=-i;
#define TURN_LEFT(i) 					set_spd[0]=-i;set_spd[1]=i;
#define TURN_RIGHT(i) 				set_spd[0]=i;set_spd[1]=-i;
#define STOP 									set_spd[0]=0;set_spd[1]=0;

uint8_t BSP_Compare_Msg(uint8_t* rx_msg);
void BSP_Move_Ps2_Control(void);
void BSP_Move_Ps2_Scan_Command(void);

#endif
