#include "bsp_ps2_driver.h"

#define PS2_JOY_DELAY_TIME user_delay_us(5);
#define PS2_JOY_DI         (PS2_DI_GPIO_Port -> IDR & PS2_DI_Pin)
#define PS2_JOY_DO_H       (PS2_DO_GPIO_Port -> BSRR = PS2_DO_Pin)
#define PS2_JOY_DO_L       (PS2_DO_GPIO_Port -> BSRR = (uint32_t)PS2_DO_Pin << 16U)
#define PS2_JOY_CS_H       (PS2_CS_GPIO_Port -> BSRR = PS2_CS_Pin)
#define PS2_JOY_CS_L       (PS2_CS_GPIO_Port -> BSRR = (uint32_t)PS2_CS_Pin << 16U)
#define PS2_JOY_CLK_H      (PS2_CLK_GPIO_Port -> BSRR = PS2_CLK_Pin)
#define PS2_JOY_CLK_L      (PS2_CLK_GPIO_Port -> BSRR = (uint32_t)PS2_CLK_Pin << 16U)

uint16_t Handkey;	// ����ֵ��ȡ����ʱ�洢��
uint8_t Comd[2]={0x01,0x42};	//��ʼ�����������
uint8_t Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //���ݴ洢����
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
	};	//����ֵ�밴����

	
static void PS2_ShortPoll(void);
static void PS2_Cmd(uint8_t CMD);		 //���ֱ���������
static void PS2_EnterConfing(void);	 //��������
static void PS2_ExitConfing(void);	 //�������
	
	
extern uint16_t ps2_LX,ps2_LY,ps2_RX,ps2_RY,ps2_KEY;
	
void PS2_test(void)
{
	int ps2_LX,ps2_LY,ps2_RX,ps2_RY,ps2_KEY;
	//PS2_stdlib_Init();  /* cubeMX HAL ����Ҫ */
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

//�ֱ����ó�ʼ��
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	
	PS2_EnterConfing();		//��������ģʽ
	
	PS2_TurnOnAnalogMode();	//����ģʽ����
	PS2_VibrationMode();	//������ģʽ
	
	PS2_ExitConfing();		//��ɲ���������
	// ��һ����
//	PS2_Vibration(0x0,0xFF);
//	HAL_Delay(200);
//	PS2_Vibration(0x0,0x0);
	
}

//���ֱ���������
void PS2_Cmd(uint8_t CMD)
{
	volatile uint16_t ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			PS2_JOY_DO_H;                   //���һλ����λ
		}
		else PS2_JOY_DO_L;

		PS2_JOY_CLK_H;                        //ʱ������
		PS2_JOY_DELAY_TIME;
		PS2_JOY_CLK_L;
		PS2_JOY_DELAY_TIME;
		PS2_JOY_CLK_H;
		if(PS2_JOY_DI)
			Data[1] = ref|Data[1];
	}
	user_delay_us(16);
}

//�ж��Ƿ�Ϊ���ģʽ,0x41=ģ���̵ƣ�0x73=ģ����
//����ֵ��0�����ģʽ
//		  ����������ģʽ
uint8_t PS2_RedLight(void)
{
	PS2_JOY_CS_L;
	PS2_Cmd(Comd[0]);  //��ʼ����
	PS2_Cmd(Comd[1]);  //��������
	PS2_JOY_CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
//��ȡ�ֱ�����
void PS2_ReadData(void)
{
	volatile uint8_t byte=0;
	volatile uint16_t ref=0x01;
	PS2_JOY_CS_L;
	PS2_Cmd(Comd[0]);  //��ʼ����
	PS2_Cmd(Comd[1]);  //��������

	for(byte=2;byte<9;byte++)          //��ʼ��������
	{
		Data[byte]= 0;  /* �������㣬���ǵ��γ��ֱ�������0λֵ���� */
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
	
	/*����γ���������������ȫ0����Ҫ������0λ */
	if(Data[4] == 0 && Data[3] == 0)
	{
		Data[4] = Data[3] = 0xff;
	}
	if(Data[5] == 0 && Data[6] == 0 && Data[7] == 0 && Data[8] == 0)
	{
		Data[5] = Data[7] = 128;
		Data[6] = Data[8] = 127;
	}
	/* ��������ֲ��� */
	if(Data[5] == 0xff && Data[6] == 0xff && Data[7] == 0xff && Data[8] == 0xff)
	{
		PS2_SetInit();
		Data[5] = Data[7] = 128;
		Data[6] = Data[8] = 127;
	}
}

//�Զ�������PS2�����ݽ��д���,ֻ����������  
//ֻ��һ����������ʱ����Ϊ0�� δ����Ϊ1
uint16_t PS2_getKey()
{
//	uint8_t index;

	Handkey=(Data[4]<<8)|Data[3];     //����16������  ����Ϊ0�� δ����Ϊ1
	Handkey = ~Handkey;

	return Handkey;
	
	/* ������� 1~16�� ģʽ*/
	/*
	for(index=0;index<16;index++)
	{	    
		if((Handkey & KEY_MASK[index])!=0)
		return index + 1;
	}
	return 0;          //û���κΰ�������
	*/
}

//�õ�һ��ҡ�˵�ģ����	 ��Χ0~256
uint8_t PS2_getRockerAnolog(uint8_t RSS_POS)
{
	return Data[RSS_POS];
}

/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: �ֱ��𶯺�����
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:�Ҳ�С�𶯵�� 0x00�أ�������
	   motor2:�����𶯵�� 0x40~0xFF �������ֵԽ�� ��Խ��
******************************************************/
void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
	PS2_JOY_CS_L;
	user_delay_us(16);
  PS2_Cmd(0x01);  //��ʼ����
	PS2_Cmd(0x42);  //��������
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
//��������
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
//����ģʽ����
void PS2_TurnOnAnalogMode(void)
{
	PS2_JOY_CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  ������÷���ģʽ�� ���ģʽ
	PS2_Cmd(0x03); //Ox03�������ã�������ͨ��������MODE������ģʽ��
								//0xEE������������ã���ͨ��������MODE������ģʽ��
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_JOY_CS_H;
	user_delay_us(16);
}
//������
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
//��ɲ���������
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
