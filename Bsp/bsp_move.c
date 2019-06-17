#include "includes.h"

extern uint8_t compare_msg[5];
extern uint8_t last_msg;
extern uint8_t move_msg[3];
extern uint8_t ret_msg;

extern double move_target_spd;
extern double turn_target_spd;

extern double move_max_spd;
extern double turn_max_spd;
extern double move_min_spd;
extern double turn_min_spd;

extern uint16_t ps2_LX,ps2_LY,ps2_RX,ps2_RY,ps2_KEY;

extern float set_spd[2];

/**********************************************
Function name:   BSP_Compare_Msg
Features:        比较串口接收到的消息
Parameter:       rx_msg---待比较消息指针
Return value:    last_msg---比较所得结果
**********************************************/
uint8_t BSP_Compare_Msg(uint8_t* rx_msg)
{
	
	int i=0;
	for(i=0;i<3;i++)
	{
		if(rx_msg[i] != rx_msg[i+1])
		{
			return last_msg;
		}
	}
	for(i=0;i<5;i++)
	{
		if(rx_msg[0] == compare_msg[i])
		{
			last_msg = compare_msg[i];
			return compare_msg[i];
		}
	}
	return last_msg;
	
	
//	//uint8_t ret_msg=0x00;
//	move_msg[2]=rx_msg[0];
//	if(move_msg[1]==move_msg[0] && move_msg[1]==move_msg[2])
//	{
//		ret_msg = move_msg[1];
//	}
//	else if(move_msg[0]==move_msg[2])
//	{
//		move_msg[1]=move_msg[0];
//		ret_msg=move_msg[1];
//	}
//	else
//	{
//		move_msg[2]=move_msg[1]=move_msg[0];
//		ret_msg=move_msg[0];
//	}
//	return ret_msg;
	
}

/**********************************************
Function name:   BSP_Control
Features:        处理ps2接收到的消息，控制小车运动
Parameter:       无
Return value:    无
**********************************************/
void BSP_Move_Ps2_Control(void)
{
	
	switch(ps2_KEY)
	{
		case PS2_JOY_PSB_NONE:
			STOP;
			break;
		case PS2_JOY_PSB_GREEN:   
			if(move_target_spd<move_max_spd) move_target_spd++;
			break;
		case PS2_JOY_PSB_BLUE:
			if(move_target_spd>move_min_spd) move_target_spd--;
			break;
		
		case PS2_JOY_PSB_PINK:
			if(turn_target_spd<turn_max_spd) turn_target_spd++;
			break;
		case PS2_JOY_PSB_RED:     
			if(turn_target_spd>turn_min_spd) turn_target_spd--;
			break;
		
		case PS2_JOY_PSB_PAD_UP:
			MOVE_FORWARD(move_target_spd);break;
		case PS2_JOY_PSB_PAD_DOWN:
			MOVE_BACK(move_target_spd);break;
		case PS2_JOY_PSB_PAD_LEFT:
			TURN_LEFT(turn_target_spd);break;
		case PS2_JOY_PSB_PAD_RIGHT:
			TURN_RIGHT(turn_target_spd);break;
	}
	//printf("%x...move:%lf...turn:%lf\n",ps2_KEY,move_target_spd,turn_target_spd);
	
}


/**********************************************
Function name:   BSP_Move_Ps2_Scan_Command
Features:        扫描ps2遥控数据
Parameter:       无
Return value:    无，直接操作全局变量ps2_KEY
**********************************************/
void BSP_Move_Ps2_Scan_Command(void)
{
	PS2_ReadData();
	ps2_KEY=PS2_getKey();
}
