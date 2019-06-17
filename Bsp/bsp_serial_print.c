#include "includes.h"

//这个变量是其他.c文件定义的
extern UART_HandleTypeDef huart3;
uint8_t ch;
uint8_t ch_r;

//重写这个函数,重定向printf函数到串口，意思就是说printf直接输出到串口，其默认输出到控制台的
/*fputc*/
/**********************************************
Function name:   fputc
Features:        重定向printf
Parameter:       c---字符
								 f---无作用
Return value:    c
**********************************************/
int fputc(int c, FILE * f)
{
    ch=c;
    HAL_UART_Transmit(&huart3,&ch,1,1000);//发送串口
    return c;
}



//重定向scanf函数到串口 意思就是说接受串口发过来的数据，其默认是接受控制台的数据
/*fgetc*/
/**********************************************
Function name:   fgutc
Features:        重定向printf
Parameter:       F---无作用
Return value:    ch_r---收到的字符
**********************************************/
int fgetc(FILE * F)    
{
    HAL_UART_Receive (&huart3,&ch_r,1,0xffff);//接收
    return ch_r;
}
