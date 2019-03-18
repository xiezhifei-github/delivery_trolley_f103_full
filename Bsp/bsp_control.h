#ifndef BSP_CONTROL_H
#define BSP_CONTROL_H

#include "stm32f1xx_hal.h"

#include "gpio.h"
//#include "dma.h"
#include "i2c.h"
#include "tim.h"

#define MAX_SPD 16*80          //pwm最大占空比
#define MAX_PID_CHANGE 400   //单次pid输出最大变化值

typedef uint8_t pid_id;
typedef uint8_t k_flag;

enum k_flag
{
	k_flag_kp=1,
	k_flag_ki=2,
	k_flag_kd=3
};

enum pid_id
{
	pid_id_1=0,
	pid_id_2=1
};

typedef struct pid_struct_typedef
{
	pid_id id;
	double target;
	double current;
	
	float proportion;
	float integer;
	float differential;
	
	float err;
	float last_err;
	float pre_err;
	
	float Kp;
	float Ki;
	float Kd;
	
	float out;
	float out_last;
}moto_pid;

void BSP_Pid_Init(moto_pid *pid, pid_id id, float kp, float ki, float kd);
void BSP_Pid_SetTarget(moto_pid *pid, double target);
void BSP_Pid_SetCurrent(moto_pid *pid, double current);
void BSP_Pid_calculate(moto_pid *pid);
void BSP_Pid_Update(moto_pid *pid, k_flag flag, float value);

#endif
