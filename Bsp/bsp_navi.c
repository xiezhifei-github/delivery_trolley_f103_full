#include "includes.h"

extern navi_message navi_msg;
extern float set_spd[2];
extern float temp_v[2];
float check_spd[2];
extern double move_max_spd;
extern double move_min_spd;
//extern double set_spd_pc[2];

/**********************************************
Function name:   BSP_NAVI_Control
Features:        ͨ��navi_msg����С������ת��
Parameter:       �ޣ�ֱ�Ӳ���ȫ�ֱ���navi_msg
Return value:    �ޣ�ֱ�Ӳ���ȫ�ֱ���set_spd
**********************************************/
void BSP_NAVI_Control(void)
{
	//�����������ٶ�
	temp_v[0]=navi_msg.linear.data_double - 0.5*navi_msg.angular.data_double/2;
	temp_v[1]=navi_msg.linear.data_double + 0.5*navi_msg.angular.data_double/2;
	
	//��������Ŀ����� 
	check_spd[0]=temp_v[0]*1.0/(125.0/1000/2)/(2*3.1416)*13*51;
	check_spd[1]=temp_v[1]*1.0/(125.0/1000/2)/(2*3.1416)*13*51;
	
	//if(check_spd[0] < move_max_spd && check_spd[0] > move_max_spd && check_spd[1] < move_max_spd && check_spd[1] > move_max_spd)
	{
		set_spd[0]=check_spd[0];
		set_spd[1]=check_spd[1];
	}
	//printf("%lf...%lf.\n",set_spd[0],set_spd[1]);
}
