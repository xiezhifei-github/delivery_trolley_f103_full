#include "bsp_delay.h"

#include "stm32f1xx_hal.h"


/**********************************************
Function name:   user_delay_us
Features:        �û���ʱ������us��
Parameter:       us---��ʱʱ�䣬��λus
Return value:    ��
**********************************************/
void user_delay_us(uint32_t us)
{
	us *= 4;         /* 16MHz Systick */
	while(us--)
	{
		__nop();
	}
}
