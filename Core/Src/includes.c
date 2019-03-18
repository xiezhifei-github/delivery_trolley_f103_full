/* User Code Begin Include Files ********************/
#include "includes.h"
/* User Code End Include Files **********************/

/* Global Variables *********************************/

/* Variables Used in main.c File Begin***************/

//ʹ��JY901ʱʹ�õ�imu����
//uint8_t imu_data[24];          //��¼imuԪ����
//uint8_t raw_height[2];
//float acc[3];                  //imu���ٶ�
//float omiga[3];                //imu���ٶ�
//float magnetic[3];              
//float angle[3];                //imu�Ƕ�
//float height;                  //imu�߶�
//float omiga_z;                 //z����ٶ�

//ʹ��sshң��ʱ������ͨѶ���õ��ı���
int_u8_union rx_spd;           //���ڽ��ܵ����ݿ��Ƶ��ת��
//double_u8_union tx_linear,tx_angular;
tx_message tx_msg;      //��С�����͵���Ϣ
uint8_t rx_ctrl_msg[4];        //���յĿ�����Ϣ
uint8_t debug_flag=0;         //���ƴ��ڷ��͵��������ͣ�rx_ctrl_msg�յ� 04 04 04 04 Ϊdebugģʽ��printf,debug_flag=1����05 05 05 05 Ϊworkģʽ��UART_TRANSMIT,debug_flag=0��


//pid����ʱ�ı���
moto_pid bsp_pid[2];     //pid�ṹ
int set_spd[2]={0, 0};   //�趨Ŀ���ٶ� 13*51��ʾ 1 r/s����Ҫ�ı��ٶ�ֱ�Ӹı�˱���
double last_encoder[2]={0,0};    //��¼�ϴα�������ֵ���˲�
double now_encoder[2];					 //��¼��ǰ��������ֵ���˲�
double current_spd[2];           //��ǰ�ٶ�ԭʼ����
double last_spd[2];              //
double speed_lpf[2];             //�˲�����ٶȣ���΢����������ļ��Ǵ�����

float lpf_index = 0.18;          //�����˲���������ԽС��Խƽ����������ӦԽman  lpf=0.25
double move_target_spd=300;  //ֱ���˶�ʱ��Ŀ���ٶ�
double turn_target_spd=150;  //ת��ʱ��Ŀ���ٶȣ����ϸ���������bsp_move.h�е���

//TIM�жϿ���ʱ�ı���
uint8_t msg[6]={0x00,0x01,0x02,0x03,0x04,0x05};
uint8_t cnt=0;                   //����Ƶ��ѭ������
uint8_t control_period=6;        //����Ƶ��=control_period*10ms
uint8_t set_pwm_period=1;        
uint8_t init_flags=1;

//ʹ��mpu6050ʱ�õ��ı���
short Accel[3];                  //��¼���ٶ�ԭʼ����
float Accel_g[3];                //��¼���ٶ���gΪ��λʱ������
short Gyro[3];                   //��¼���ٶ�ԭʼ����
long Quat[4];                    //��¼��Ԫ��ԭʼ����
float Omiga[3];                  //��¼���ٶ��Ի���Ϊ��λ�����ݣ�����ʱʹ�ã�
short Temp;
unsigned long sensor_timestamp;  //������ʱ���
unsigned char more;              //
short sensors;
float gyro;                      //��¼���ٶ��Ի���Ϊ��λ�����ݣ���������ʱʹ�ã�
float gyro_zero=0;                 //��¼���Ư��
uint8_t gyro_num = 200;                //���Ư������

int result;                      //��ʼ�����
struct int_param_s int_param_s;  //��ʼ���ṹ��

//�������˲�ʹ�õ��ı���

kalman1_state kalman_state;      //�������ṹ��
float kalman_x=1.0;
float kalman_p=1.0;
float kalman_result=0.0;



/* Variables Used in main.c File End*****************/


/* Variables Used in bsp_contral.c Files Begin *****/
double pid_lpf=0.6;     //pid����˲�ϵ��


/* Variables Used in bsp_move.c File Begin **********/
//ps2ң࠘ԃսքҤ
uint16_t ps2_LX,ps2_LY,ps2_RX,ps2_RY,ps2_KEY;

uint8_t compare_msg[5]={0x00,0x01,0x02,0x03,0x04};
uint8_t last_msg=0x00;
uint8_t move_msg[3]={0x00,0x00,0x00};
uint8_t ret_msg=0x00;

double move_max_spd=1000;
double turn_max_spd=1000;
double move_min_spd=200;
double turn_min_spd=200;

/* Variables Used in bsp_move.c File End ************/


