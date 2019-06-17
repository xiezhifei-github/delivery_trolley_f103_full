#include "includes.h"

extern uint8_t buff[38];
extern navi_message navi_msg;

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


/**********************************************
Function name:   BSP_UART_Receive
Features:        串口接收navi_msg
Parameter:       无，直接操作全局变量buff和navi_msg
Return value:    无
**********************************************/
void BSP_UART_Receive()
{
	for(int i=0;i<22;i++)
	{
		if(buff[i]==0xAA && buff[i+9]==0xA0)
		{
			navi_msg.linear.data_u8[0]=buff[i+1];
			navi_msg.linear.data_u8[1]=buff[i+2];
			navi_msg.linear.data_u8[2]=buff[i+3];
			navi_msg.linear.data_u8[3]=buff[i+4];
			navi_msg.linear.data_u8[4]=buff[i+5];
			navi_msg.linear.data_u8[5]=buff[i+6];
			navi_msg.linear.data_u8[6]=buff[i+7];
			navi_msg.linear.data_u8[7]=buff[i+8];
			
			navi_msg.angular.data_u8[0]=buff[i+9+1];
			navi_msg.angular.data_u8[1]=buff[i+9+2];
			navi_msg.angular.data_u8[2]=buff[i+9+3];
			navi_msg.angular.data_u8[3]=buff[i+9+4];
			navi_msg.angular.data_u8[4]=buff[i+9+5];
			navi_msg.angular.data_u8[5]=buff[i+9+6];
			navi_msg.angular.data_u8[6]=buff[i+9+7];
			navi_msg.angular.data_u8[7]=buff[i+9+8];
		}

	}
}

/**********************************************
Function name:   BSP_UART_DataLimit
Features:        对串口将要发送的数据限幅处理
Parameter:       无，直接操作全局变量tx_msg
Return value:    无
**********************************************/
void BSP_UART_DataLimit(tx_message *tx_msg)
{
	//角速度限制为2
	if(tx_msg->angular.data_float > 2)
	{
		tx_msg->angular.data_float = 2;
	}
	if(tx_msg->angular.data_float < -2)
	{
		tx_msg->angular.data_float = -2;
	}
	
	//线速度限制为10
	if(tx_msg->linear.data_float > 10)
	{
		tx_msg->linear.data_float = 10;
	}
	if(tx_msg->linear.data_float < -10)
	{
		tx_msg->linear.data_float = -10;
	}
}
