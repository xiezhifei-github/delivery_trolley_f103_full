#include "bsp_delay.h"

#include "stm32f1xx_hal.h"


/**********************************************
Function name:   user_delay_us
Features:        用户延时函数，us级
Parameter:       us---延时时间，单位us
Return value:    无
**********************************************/
void user_delay_us(uint32_t us)
{
	us *= 4;         /* 16MHz Systick */
	while(us--)
	{
		__nop();
	}
}
