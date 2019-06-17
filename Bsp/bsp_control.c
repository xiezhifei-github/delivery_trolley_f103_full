#include "includes.h"

extern double pid_lpf;

/*************************************************
 * Function name:   BSP_Pid_Init
 * Features:        pid结构体初始化，赋值Kp、Ki、Kd
 * Parameter:       pid---pid结构体指针
 *                  id---pid标识
 * 								 kp---P
 * 								 ki---i
 * 								 kd---d
 * Return value:    无
*************************************************/
void BSP_Pid_Init(moto_pid *pid, pid_id id, float kp, float ki, float kd)
{
	pid->id = id;
	
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	
	pid->err = 0;
	pid->integer = 0;
	pid->last_err = 0;
	pid->pre_err = 0;
	
	pid->out = 0;
	pid->out_last = 0;
}

/**********************************************
 * Function name:   BSP_Pid_SetTarget
 * Features:        设定pid的目标值
 * Parameter:       pid---pid结构体指针
 *                  target---目标值
 * Return value:    无
*************************************************/
void BSP_Pid_SetTarget(moto_pid *pid, double target)
{
	pid->target=target;
}

/*************************************************
 * Function name:   BSP_Pid_SetCurrent
 * Features:        设定pid的当前值
 * Parameter:       pid---pid结构体指针
 *                  current---当前值
 * Return value:    无
*************************************************/
void BSP_Pid_SetCurrent(moto_pid *pid, double current)
{
	if(current>-20000&&current<20000)
		pid->current=current;
}

/*************************************************
 * Function name:   BSP_Pid_calculate
 * Features:        计算pid输出
 * Parameter:       pid---pid结构体指针
 * Return value:    无
*************************************************/
void BSP_Pid_calculate(moto_pid *pid)
{
	//位置式pid
	pid->err=pid->target-pid->current;
	
	pid->proportion=pid->err-pid->last_err;
	
	if(pid->err > -150 && pid->err < 150)
		pid->integer +=pid->err;  
	if(pid->integer > 1000)
		pid->integer  = 1000;
	if(pid->integer < -1000)
		pid->integer  = -1000;
	
	pid->differential=pid->err-pid->last_err;
	
	//if(pid->err>10||pid->err<-10)
		pid->out = pid->Kp* pid->err + pid->Ki*pid->integer + pid->Kd*pid->differential;
	
	pid->pre_err=pid->last_err;
	pid->last_err=pid->err;

	
	pid->out=pid_lpf*pid->out +(1-pid_lpf)*pid->out_last;
		
	//控制单次pid输出的变化不超过MAX_PID_CHANGE，，，，使平滑
	if(pid->out-pid->out_last > MAX_PID_CHANGE)
		pid->out=pid->out_last+MAX_PID_CHANGE;
	if(pid->out-pid->out_last < -MAX_PID_CHANGE)
		pid->out=pid->out_last-MAX_PID_CHANGE;
	pid->out_last=pid->out;
	
	//输出限幅
	if(pid->out>MAX_SPD)
		pid->out=MAX_SPD;
	if(pid->out<-MAX_SPD)
		pid->out=-MAX_SPD;
}
