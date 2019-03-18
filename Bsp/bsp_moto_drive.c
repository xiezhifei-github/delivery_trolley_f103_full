#include "includes.h"

//uint32_t BSP_Moto_GetSpeed(MotoIDTypeDef moto_id)
//{
//	uint32_t enc1=0,enc2=0;
//	uint32_t spd=0;
//	
//	
//	switch(moto_id)
//	{
//		case moto_ID_1:
//			enc1=BSP_Encoder_GetValue(&htim2);
//			HAL_Delay(5);
//			enc2=BSP_Encoder_GetValue(&htim2);
//			break;
//		case moto_ID_2:
//			enc1=BSP_Encoder_GetValue(&htim3);
//			HAL_Delay(5);
//			enc2=BSP_Encoder_GetValue(&htim3);
//			break;
//	}

//	

//		spd=(enc2-enc1)*200;
//	return spd;
//}

extern uint8_t control_period;

double old_value[2];
double new_value[2];

/**********************************************
Function name:   BSP_Moto_GetValue
Features:        根据 moto_id 获取编码器读数
Parameter:       moto_id---电机ID，参考 MotoIDTypeDef
Return value:    无
**********************************************/
uint32_t BSP_Moto_GetValue(MotoIDTypeDef moto_id)
{
	uint32_t value;
	switch(moto_id)
	{
		case moto_ID_1:
			value=__HAL_TIM_GET_COUNTER(&htim3);
		
			break;
		case moto_ID_2:
			value=__HAL_TIM_GET_COUNTER(&htim4);
			break;
	}
	return value;
}

/**********************************************
Function name:   BSP_Moto_SetSpeed
Features:        设定指定电机的pwm值
Parameter:       moto_id---电机ID，参考 MotoIDTypeDef
                 direction---转动方向，参考DirectionTypeDef
                 value---pwm值，范围在0~1600-1（Period）之间
Return value:    无
**********************************************/
void BSP_Moto_SetSpeed(MotoIDTypeDef moto_id, DirectionTypeDef direction, uint32_t value)
{
	switch(moto_id)
	{
		case moto_ID_1:
			if(direction==moto_direction_forward)
			{
				BSP_PWM_SetValue(&htim8,TIM_CHANNEL_1,value);
				BSP_PWM_SetValue(&htim8,TIM_CHANNEL_2,0);
			}
			else
			{
				BSP_PWM_SetValue(&htim8,TIM_CHANNEL_1,0);
				BSP_PWM_SetValue(&htim8,TIM_CHANNEL_2,value);
			}
			break;
		case moto_ID_2:
			if(direction==moto_direction_forward)
			{
				BSP_PWM_SetValue(&htim8,TIM_CHANNEL_3,value);
				BSP_PWM_SetValue(&htim8,TIM_CHANNEL_4,0);
			}
			else
			{
				BSP_PWM_SetValue(&htim8,TIM_CHANNEL_3,0);
				BSP_PWM_SetValue(&htim8,TIM_CHANNEL_4,value);
			}
			break;
	}
}

/**********************************************
Function name:   BSP_Moto_Control
Features:        设定两个电机转速
Parameter:       value1---电机1 pwm 值，在-1599~1599之间（Period）
                 value1---电机2 pwm 值，在-1599~1599之间
Return value:    无
**********************************************/
void BSP_Moto_Control(double value1, double value2)
{
	value1>0.0?BSP_Moto_SetSpeed(moto_ID_1,moto_direction_forward,(uint32_t)value1):BSP_Moto_SetSpeed(moto_ID_1,moto_direction_back,(uint32_t)-value1);
	value2>0.0?BSP_Moto_SetSpeed(moto_ID_2,moto_direction_forward,(uint32_t)value2):BSP_Moto_SetSpeed(moto_ID_2,moto_direction_back,(uint32_t)-value2);
}

/**********************************************
Function name:   BSP_Moto_SpeedControlOutput
Features:        线性插值pid.out，解决启动震颤的问题
Parameter:       pid---待插值pid的结构体
                 cnt---当前计数值
Return value:    无
**********************************************/
void BSP_Moto_SpeedControlOutput(moto_pid *pid, uint8_t cnt)
{
	pid->out+=(pid->target-pid->out)/control_period*(cnt+1);
}

