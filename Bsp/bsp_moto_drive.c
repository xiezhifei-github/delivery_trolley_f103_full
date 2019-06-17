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
Features:        ���� moto_id ��ȡ����������
Parameter:       moto_id---���ID���ο� MotoIDTypeDef
Return value:    ��
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
Features:        �趨ָ�������pwmֵ
Parameter:       moto_id---���ID���ο� MotoIDTypeDef
                 direction---ת�����򣬲ο�DirectionTypeDef
                 value---pwmֵ����Χ��0~1600-1��Period��֮��
Return value:    ��
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
Features:        �趨�������ת��
Parameter:       value1---���1 pwm ֵ����-1599~1599֮�䣨Period��
                 value1---���2 pwm ֵ����-1599~1599֮��
Return value:    ��
**********************************************/
void BSP_Moto_Control(double value1, double value2)
{
	value1>0.0?BSP_Moto_SetSpeed(moto_ID_1,moto_direction_forward,(uint32_t)value1):BSP_Moto_SetSpeed(moto_ID_1,moto_direction_back,(uint32_t)-value1);
	value2>0.0?BSP_Moto_SetSpeed(moto_ID_2,moto_direction_forward,(uint32_t)value2):BSP_Moto_SetSpeed(moto_ID_2,moto_direction_back,(uint32_t)-value2);
}

/**********************************************
Function name:   BSP_Moto_SpeedControlOutput
Features:        ���Բ�ֵpid.out������������������
Parameter:       pid---����ֵpid�Ľṹ��
                 cnt---��ǰ����ֵ
Return value:    ��
**********************************************/
void BSP_Moto_SpeedControlOutput(moto_pid *pid, uint8_t cnt)
{
	pid->out+=(pid->target-pid->out)*1.0/control_period*(cnt+1);
}
