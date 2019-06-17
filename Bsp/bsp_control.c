#include "includes.h"

extern double pid_lpf;

/*************************************************
 * Function name:   BSP_Pid_Init
 * Features:        pid�ṹ���ʼ������ֵKp��Ki��Kd
 * Parameter:       pid---pid�ṹ��ָ��
 *                  id---pid��ʶ
 * 								 kp---P
 * 								 ki---i
 * 								 kd---d
 * Return value:    ��
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
 * Features:        �趨pid��Ŀ��ֵ
 * Parameter:       pid---pid�ṹ��ָ��
 *                  target---Ŀ��ֵ
 * Return value:    ��
*************************************************/
void BSP_Pid_SetTarget(moto_pid *pid, double target)
{
	pid->target=target;
}

/*************************************************
 * Function name:   BSP_Pid_SetCurrent
 * Features:        �趨pid�ĵ�ǰֵ
 * Parameter:       pid---pid�ṹ��ָ��
 *                  current---��ǰֵ
 * Return value:    ��
*************************************************/
void BSP_Pid_SetCurrent(moto_pid *pid, double current)
{
	if(current>-20000&&current<20000)
		pid->current=current;
}

/*************************************************
 * Function name:   BSP_Pid_calculate
 * Features:        ����pid���
 * Parameter:       pid---pid�ṹ��ָ��
 * Return value:    ��
*************************************************/
void BSP_Pid_calculate(moto_pid *pid)
{
	//λ��ʽpid
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
		
	//���Ƶ���pid����ı仯������MAX_PID_CHANGE��������ʹƽ��
	if(pid->out-pid->out_last > MAX_PID_CHANGE)
		pid->out=pid->out_last+MAX_PID_CHANGE;
	if(pid->out-pid->out_last < -MAX_PID_CHANGE)
		pid->out=pid->out_last-MAX_PID_CHANGE;
	pid->out_last=pid->out;
	
	//����޷�
	if(pid->out>MAX_SPD)
		pid->out=MAX_SPD;
	if(pid->out<-MAX_SPD)
		pid->out=-MAX_SPD;
}
