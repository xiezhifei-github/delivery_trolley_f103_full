#include "bsp_delay.h"

#include "stm32f1xx_hal.h"

void user_delay_us(uint32_t us)
{
	us *= 4;         /* 16MHz Systick */
	while(us--)
	{
		__nop();
	}
}
