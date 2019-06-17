/* User Code Begin Include Files ********************/
#include "includes.h"
/* User Code End Include Files **********************/

/* Global Variables *********************************/

/* Variables Used in main.c File Begin***************/

uint32_t tick;                   //记录系统运行时间，在主循环中使用，控制信息发送周期

//使用JY901时使用的imu变量
//uint8_t imu_data[24];          //记录imu元数据
//uint8_t raw_height[2];
//float acc[3];                  //imu加速度
//float omiga[3];                //imu角速度
//float magnetic[3];             //磁场
//float angle[3];                //imu角度
//float height;                  //imu高度
//float omiga_z;                 //z轴角速度

//使用ssh遥控时（串口通讯）用到的变量
int_u8_union rx_spd;             //串口接受的数据控制电机转速
//double_u8_union tx_linear,tx_angular;
tx_message tx_msg;               //向小车发送的信息
uint8_t rx_ctrl_msg[4];          //接收的控制信息
uint8_t debug_flag = 0;          //控制串口发送的数据类型，rx_ctrl_msg收到 04 04 04 04 为debug模式（printf,debug_flag = 1），05 05 05 05 为work模式（UART_TRANSMIT,debug_flag = 0），//未使用


//pid控制时的变量
moto_pid bsp_pid[2];             //pid结构
float set_spd[2] = {0, 0};       //设定目标速度 13*51表示 1 r/s，想要改变速度直接改变此变量，一般由程序改变
double set_spd_ps2[2];
double set_spd_pc[2];
double last_encoder[2] = {0,0};  //记录上次编码器数值，滤波
double now_encoder[2];					 //记录当前编码器数值，滤波
double current_spd[2];           //当前速度原始数据
double last_spd[2];              //
double speed_lpf[2];             //滤波后的速度，向微型主机传输的即是此数据

float lpf_index  =  0.18;        //后置滤波器变量，越小，越平滑，但是相应越man  lpf = 0.25
double move_target_spd = 300;    //直线运动时的目标速度
double turn_target_spd = 150;    //转弯时的目标速度，与上个变量均在bsp_move.h中调用

//TIM中断遥控控制时的变量
uint8_t msg[6] = {0x00,0x01,0x02,0x03,0x04,0x05};
uint8_t cnt = 0;                 //控制频率循环变量
uint8_t control_period = 6;      //控制频率 = control_period*10ms
uint8_t ps2_cnt = 0;
uint8_t set_pwm_period = 1;        
uint8_t init_flags = 1;
//uart中断接收
uint8_t buff[38];

//使用mpu6050时用到的变量
short Accel[3];                  //记录加速度原始数据
float Accel_g[3];                //记录加速度以g为单位时的数据
short Gyro[3];                   //记录角速度原始数据
long Quat[4];                    //记录四元数原始数据
float Omiga[3];                  //记录角速度以弧度为单位的数据（测试时使用）
short Temp;
unsigned long sensor_timestamp;  //传感器时间戳
unsigned char more;              //
short sensors;
float gyro;                      //记录角速度以弧度为单位的数据（正常工作时使用）
float gyro_zero = 0;             //记录零点漂移
uint8_t gyro_num = 200;          //零点漂移数量

int result;                      //初始化结果
struct int_param_s int_param_s;  //初始化结构体

//卡尔曼滤波使用到的变量

kalman1_state kalman_state;      //卡尔曼结构体
float kalman_x = 1.0;
float kalman_p = 1.0;
float kalman_result = 0.0;

/* Variables Used in main.c File End*****************/


/* Variables Used in bsp_contral.c Files Begin *****/
double pid_lpf = 0.6;     //pid输出滤波系数


/* Variables Used in bsp_move.c File Begin **********/
//ps2遥控用到的变量
uint16_t ps2_LX,ps2_LY,ps2_RX,ps2_RY,ps2_KEY;

uint8_t compare_msg[5] = {0x00,0x01,0x02,0x03,0x04};
uint8_t last_msg = 0x00;
uint8_t move_msg[3] = {0x00,0x00,0x00};
uint8_t ret_msg = 0x00;

double move_max_spd = 1000.0;
double turn_max_spd = 1000.0;
double move_min_spd = 200.0;
double turn_min_spd = 200.0;

/* Variables Used in bsp_move.c File End ************/


/* Variables Used in bsp_navi.c File Begin **********/
navi_message navi_msg;
double temp_v[2];
/* Variables Used in bsp_navi.c File End ************/


/* Variables Used in bsp_lock.c File Begin **********/
uint8_t lock_status = LOCK_CLOSE;
/* Variables Used in bsp_lock.c File End ************/

/* Variables Used in bsp_mode.c File Begin **********/
control_mode mode;
/* Variables Used in bsp_mode.c File End ************/
