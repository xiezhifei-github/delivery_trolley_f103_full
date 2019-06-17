#include "bsp_mode.h"

extern control_mode mode;

/**********************************************
Function name:   BSP_Mode_Control
Features:        switch控制模式
Parameter:       无，直接操作全局变量mode
Return value:    无
**********************************************/
void BSP_Mode_Control(void)
{
	GPIO_PinState bitstatus = HAL_GPIO_ReadPin(MODE_SELECT_GPIO_Port,MODE_SELECT_Pin);
	
	if(bitstatus == GPIO_PIN_SET)
	{
		mode = control_mode_ps2;
	}
	else
	{
		mode = control_mode_pc;
	}
}
