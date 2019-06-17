#ifndef BSP_PS2_DRIVER_H
#define BSP_PS2_DRIVER_H

#include "stm32f1xx_hal.h"
#include "bsp_delay.h"
#include "usart.h"
#include "bsp_serial_print.h"
#include "main.h"

/****************************************
#define PS2_DO_Pin GP         IO_PIN_1
#define PS2_DO_GPIO_Port      GPIOC
#define PS2_DI_Pin            GPIO_PIN_2
#define PS2_DI_GPIO_Port      GPIOC
#define PS2_CS_Pin            GPIO_PIN_3
#define PS2_CS_GPIO_Port      GPIOC
#define PS2_CLK_Pin           GPIO_PIN_4
#define PS2_CLK_GPIO_Port     GPIOA
****************************************/

/*******************************************************************/

//These are our button constants
#define PS2_JOY_PSB_NONE        0x0
#define PS2_JOY_PSB_SELECT      0x1
#define PS2_JOY_PSB_L3          0x2
#define PS2_JOY_PSB_R3          0x4
#define PS2_JOY_PSB_START       0x8
#define PS2_JOY_PSB_PAD_UP      0x10
#define PS2_JOY_PSB_PAD_RIGHT   0x20
#define PS2_JOY_PSB_PAD_DOWN    0x40
#define PS2_JOY_PSB_PAD_LEFT    0x80
#define PS2_JOY_PSB_L2          0x100
#define PS2_JOY_PSB_R2          0x200
#define PS2_JOY_PSB_L1          0x400
#define PS2_JOY_PSB_R1          0x800
#define PS2_JOY_PSB_GREEN       0x1000
#define PS2_JOY_PSB_RED         0x2000
#define PS2_JOY_PSB_BLUE        0x4000
#define PS2_JOY_PSB_PINK        0x8000

#define PS2_JOY_PSB_TRIANGLE    0x1000
#define PS2_JOY_PSB_CIRCLE      0x2000
#define PS2_JOY_PSB_CROSS       0x4000
#define PS2_JOY_PSB_SQUARE      0x8000

//#define WHAMMY_BAR		8

//These are stick values
#define PS2_JOY_PSS_RX 5                //右摇杆X轴数据位
#define PS2_JOY_PSS_RY 6
#define PS2_JOY_PSS_LX 7
#define PS2_JOY_PSS_LY 8

void PS2_test(void);

void PS2_SetInit(void);		     //配置初始化

void PS2_ReadData(void); //读手柄数据
uint16_t PS2_getKey(void);		  //读键值
uint8_t PS2_getRockerAnolog(uint8_t RSS_POS); //得到一个摇杆的模拟量

void PS2_VibrationMode(void);    //振动模式设置
void PS2_Vibration(uint8_t motor1, uint8_t motor2);//振动设置motor1  0x1开，0关，motor2  0x41~0xFF开关控制强度

void PS2_TurnOnAnalogMode(void); //发送模式设置
uint8_t PS2_RedLight(void);   //判断是否为红灯模式

void PS2_stdlib_Init(void);

//void user_scan_ps_joy(float& liner_vel_x, float& liner_vel_y, float& angular_rad_z);
#endif
