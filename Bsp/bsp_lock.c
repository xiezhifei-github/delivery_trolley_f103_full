#include "bsp_lock.h"

extern uint8_t lock_status;

//void BSP_Lock_Open()
//{
//	HAL_GPIO_WritePin(LOCK_GPIO_Port,LOCK_Pin,GPIO_PIN_RESET);
//}

/**********************************************
Function name:   BSP_Lock_Control
Features:        �������Ŀ���
Parameter:       �ޣ�ֱ�Ӳ���ȫ�ֱ���lock_status
Return value:    ��
**********************************************/
void BSP_Lock_Control()
{
	switch(lock_status)
	{
		case LOCK_OPEN:
			BSP_Lock_Open();break;
		case LOCK_CLOSE:
			BSP_Lock_Close();break;
	}
}
