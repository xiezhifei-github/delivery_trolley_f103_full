#include "bsp_ps2_driver.h"

#define PS2_JOY_DELAY_TIME user_delay_us(5);
#define PS2_JOY_DI         (PS2_DI_GPIO_Port -> IDR & PS2_DI_Pin)
#define PS2_JOY_DO_H       (PS2_DO_GPIO_Port -> BSRR = PS2_DO_Pin)
#define PS2_JOY_DO_L       (PS2_DO_GPIO_Port -> BSRR = (uint32_t)PS2_DO_Pin << 16U)
#define PS2_JOY_CS_H       (PS2_CS_GPIO_Port -> BSRR = PS2_CS_Pin)
#define PS2_JOY_CS_L       (PS2_CS_GPIO_Port -> BSRR = (uint32_t)PS2_CS_Pin << 16U)
#define PS2_JOY_CLK_H      (PS2_CLK_GPIO_Port -> BSRR = PS2_CLK_Pin)
#define PS2_JOY_CLK_L      (PS2_CLK_GPIO_Port -> BSRR = (uint32_t)PS2_CLK_Pin << 16U)

uint16_t Handkey;	// 按键值读取，零时存储。
uint8_t Comd[2]={0x01,0x42};	//开始命令。请求数据
uint8_t Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组
uint16_t KEY_MASK[]={
    PS2_JOY_PSB_SELECT,
    PS2_JOY_PSB_L3,
    PS2_JOY_PSB_R3 ,
    PS2_JOY_PSB_START,
    PS2_JOY_PSB_PAD_UP,
    PS2_JOY_PSB_PAD_RIGHT,
    PS2_JOY_PSB_PAD_DOWN,
    PS2_JOY_PSB_PAD_LEFT,
    PS2_JOY_PSB_L2,
    PS2_JOY_PSB_R2,
    PS2_JOY_PSB_L1,
    PS2_JOY_PSB_R1 ,
    PS2_JOY_PSB_GREEN,
    PS2_JOY_PSB_RED,
    PS2_JOY_PSB_BLUE,
    PS2_JOY_PSB_PINK
	};	//按键值与按键明

	
static void PS2_ShortPoll(void);
static void PS2_Cmd(uint8_t CMD);		 //向手柄发送命令
static void PS2_EnterConfing(void);	 //进入配置
static void PS2_ExitConfing(void);	 //完成配置
	
	
extern uint16_t ps2_LX,ps2_LY,ps2_RX,ps2_RY,ps2_KEY;
	
void PS2_test(void)
{
	int ps2_LX,ps2_LY,ps2_RX,ps2_RY,ps2_KEY;
	//PS2_stdlib_Init();  /* cubeMX HAL 不需要 */
	PS2_SetInit();	
	
	while(1)
	{
		PS2_ReadData();
		ps2_KEY = PS2_getKey();	
		ps2_LX = PS2_getRockerAnolog(PS2_JOY_PSS_LX);    
		ps2_LY = PS2_getRockerAnolog(PS2_JOY_PSS_LY);
		ps2_RX = PS2_getRockerAnolog(PS2_JOY_PSS_RX);
		ps2_RY = PS2_getRockerAnolog(PS2_JOY_PSS_RY);
		PS2_Vibration(0x1,0x41);
		printf("PS2 LX[%3d], LY[%3d], RX[%3d], RY[%3d], KEY[%3d]\r\n"
			,ps2_LX,ps2_LY,ps2_RX,ps2_RY,ps2_KEY);
	}
}

//手柄配置初始化
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	
	PS2_EnterConfing();		//进入配置模式
	
	PS2_TurnOnAnalogMode();	//发送模式设置
	PS2_VibrationMode();	//开启震动模式
	
	PS2_ExitConfing();		//完成并保存配置
	// 震动一下下
//	PS2_Vibration(0x0,0xFF);
//	HAL_Delay(200);
//	PS2_Vibration(0x0,0x0);
	
}

//向手柄发送命令
void PS2_Cmd(uint8_t CMD)
{
	volatile uint16_t ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			PS2_JOY_DO_H;                   //输出一位控制位
		}
		else PS2_JOY_DO_L;

		PS2_JOY_CLK_H;                        //时钟拉高
		PS2_JOY_DELAY_TIME;
		PS2_JOY_CLK_L;
		PS2_JOY_DELAY_TIME;
		PS2_JOY_CLK_H;
		if(PS2_JOY_DI)
			Data[1] = ref|Data[1];
	}
	user_delay_us(16);
}

//判断是否为红灯模式,0x41=模拟绿灯，0x73=模拟红灯
//返回值；0，红灯模式
//		  其他，其他模式
uint8_t PS2_RedLight(void)
{
	PS2_JOY_CS_L;
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	PS2_JOY_CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
//读取手柄数据
void PS2_ReadData(void)
{
	volatile uint8_t byte=0;
	volatile uint16_t ref=0x01;
	PS2_JOY_CS_L;
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据

	for(byte=2;byte<9;byte++)          //开始接受数据
	{
		Data[byte]= 0;  /* 数据清零，考虑到拔除手柄，采用0位值代替 */
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			PS2_JOY_CLK_H;
			PS2_JOY_DELAY_TIME;
			PS2_JOY_CLK_L;
			PS2_JOY_DELAY_TIME;
			PS2_JOY_CLK_H;
			if(PS2_JOY_DI)Data[byte] = ref|Data[byte];
		}
    user_delay_us(16);
	}
	PS2_JOY_CS_H;
	
	/*如果拔除接收器，则数据全0，需要处理中0位 */
	if(Data[4] == 0 && Data[3] == 0)
	{
		Data[4] = Data[3] = 0xff;
	}
	if(Data[5] == 0 && Data[6] == 0 && Data[7] == 0 && Data[8] == 0)
	{
		Data[5] = Data[7] = 128;
		Data[6] = Data[8] = 127;
	}
	/* 如果拔了又插上 */
	if(Data[5] == 0xff && Data[6] == 0xff && Data[7] == 0xff && Data[8] == 0xff)
	{
		PS2_SetInit();
		Data[5] = Data[7] = 128;
		Data[6] = Data[8] = 127;
	}
}

//对读出来的PS2的数据进行处理,只处理按键部分  
//只有一个按键按下时按下为0， 未按下为1
uint16_t PS2_getKey()
{
//	uint8_t index;

	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0， 未按下为1
	Handkey = ~Handkey;

	return Handkey;
	
	/* 以下输出 1~16键 模式*/
	/*
	for(index=0;index<16;index++)
	{	    
		if((Handkey & KEY_MASK[index])!=0)
		return index + 1;
	}
	return 0;          //没有任何按键按下
	*/
}

//得到一个摇杆的模拟量	 范围0~256
uint8_t PS2_getRockerAnolog(uint8_t RSS_POS)
{
	return Data[RSS_POS];
}

/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: 手柄震动函数，
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:右侧小震动电机 0x00关，其他开
	   motor2:左侧大震动电机 0x40~0xFF 电机开，值越大 震动越大
******************************************************/
void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
	PS2_JOY_CS_L;
	user_delay_us(16);
  PS2_Cmd(0x01);  //开始命令
	PS2_Cmd(0x42);  //请求数据
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_JOY_CS_H;
	user_delay_us(16);  
}

//short poll
void PS2_ShortPoll(void)
{
	PS2_JOY_CS_L;
	user_delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	PS2_JOY_CS_H;
	user_delay_us(16);	
}
//进入配置
void PS2_EnterConfing(void)
{
  PS2_JOY_CS_L;
	user_delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_JOY_CS_H;
	user_delay_us(16);
}
//发送模式设置
void PS2_TurnOnAnalogMode(void)
{
	PS2_JOY_CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  软件设置发送模式， 红灯模式
	PS2_Cmd(0x03); //Ox03锁存设置，即不可通过按键“MODE”设置模式。
								//0xEE不锁存软件设置，可通过按键“MODE”设置模式。
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_JOY_CS_H;
	user_delay_us(16);
}
//振动设置
void PS2_VibrationMode(void)
{
	PS2_JOY_CS_L;
	user_delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	PS2_JOY_CS_H;
	user_delay_us(16);	
}
//完成并保存配置
void PS2_ExitConfing(void)
{
  PS2_JOY_CS_L;
	user_delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_JOY_CS_H;
	user_delay_us(16);
}

//void user_scan_ps_joy(uint16_t &ps2_LX,uint16_t& ps2_LY,uint16_t& ps2_RX,uint16_t& ps2_RY,uint16_t& ps2_KEY)
//{
//	//static uint32_t tick = HAL_GetTick();
//	PS2_ReadData();
//	ps2_KEY = PS2_getKey();	
//	ps2_LX = PS2_getRockerAnolog(PS2_JOY_PSS_LX);    
//	ps2_LY = PS2_getRockerAnolog(PS2_JOY_PSS_LY);
//	ps2_RX = PS2_getRockerAnolog(PS2_JOY_PSS_RX);
//	ps2_RY = PS2_getRockerAnolog(PS2_JOY_PSS_RY);
//}
