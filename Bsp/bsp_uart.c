#include "includes.h"

/**********************************************
Function name:   BSP_UART_Transmit
Features:        串口发送tx_message
Parameter:       msg---待发送消息指针
Return value:    无
**********************************************/
void BSP_UART_Transmit(tx_message* msg)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&msg->head,sizeof(msg->head),50);
	HAL_UART_Transmit(&huart1,(uint8_t *)&msg->linear.data_u8,sizeof(msg->linear),50);
	HAL_UART_Transmit(&huart1,(uint8_t *)&msg->angular.data_u8,sizeof(msg->angular),50);
}
