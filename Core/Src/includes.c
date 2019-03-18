/* User Code Begin Include Files ********************/
#include "includes.h"
/* User Code End Include Files **********************/

/* Global Variables *********************************/

/* Variables Used in main.c File Begin***************/

//Ê¹ÓÃJY901Ê±Ê¹ÓÃµÄimu±äÁ¿
//uint8_t imu_data[24];          //¼ÇÂ¼imuÔªÊı’İ
//uint8_t raw_height[2];
//float acc[3];                  //imu¼ÓËÙ¶È
//float omiga[3];                //imu½ÇËÙ¶È
//float magnetic[3];              
//float angle[3];                //imu½Ç¶È
//float height;                  //imu¸ß¶È
//float omiga_z;                 //zÖá½ÇËÙ¶È

//Ê¹ÓÃsshÒ£¿ØÊ±£¨´®¿ÚÍ¨Ñ¶£©ÓÃµ½µÄ±äÁ¿
int_u8_union rx_spd;           //´®¿Ú½ÓÊÜµÄÊı¾İ¿ØÖÆµç»ú×ªËÙ
//double_u8_union tx_linear,tx_angular;
tx_message tx_msg;      //ÏòĞ¡³µ·¢ËÍµÄĞÅÏ¢
uint8_t rx_ctrl_msg[4];        //½ÓÊÕµÄ¿ØÖÆĞÅÏ¢
uint8_t debug_flag=0;         //¿ØÖÆ´®¿Ú·¢ËÍµÄÊı¾İÀàĞÍ£¬rx_ctrl_msgÊÕµ½ 04 04 04 04 ÎªdebugÄ£Ê½£¨printf,debug_flag=1£©£¬05 05 05 05 ÎªworkÄ£Ê½£¨UART_TRANSMIT,debug_flag=0£©


//pid¿ØÖÆÊ±µÄ±äÁ¿
moto_pid bsp_pid[2];     //pid½á¹¹
int set_spd[2]={0, 0};   //Éè¶¨Ä¿±êËÙ¶È 13*51±íÊ¾ 1 r/s£¬ÏëÒª¸Ä±äËÙ¶ÈÖ±½Ó¸Ä±ä´Ë±äÁ¿
double last_encoder[2]={0,0};    //¼ÇÂ¼ÉÏ´Î±àÂëÆ÷ÊıÖµ£¬ÂË²¨
double now_encoder[2];					 //¼ÇÂ¼µ±Ç°±àÂëÆ÷ÊıÖµ£¬ÂË²¨
double current_spd[2];           //µ±Ç°ËÙ¶ÈÔ­Ê¼Êı¾İ
double last_spd[2];              //
double speed_lpf[2];             //ÂË²¨ºóµÄËÙ¶È£¬ÏòÎ¢ĞÍÖ÷»ú´«ÊäµÄ¼´ÊÇ´ËÊı¾İ

float lpf_index = 0.18;          //ºóÖÃÂË²¨Æ÷±äÁ¿£¬Ô½Ğ¡£¬Ô½Æ½»¬£¬µ«ÊÇÏàÓ¦Ô½man  lpf=0.25
double move_target_spd=300;  //Ö±ÏßÔË¶¯Ê±µÄÄ¿±êËÙ¶È
double turn_target_spd=150;  //×ªÍäÊ±µÄÄ¿±êËÙ¶È£¬ÓëÉÏ¸ö±äÁ¿¾ùÔÚbsp_move.hÖĞµ÷ÓÃ

//TIMÖĞ¶Ï¿ØÖÆÊ±µÄ±äÁ¿
uint8_t msg[6]={0x00,0x01,0x02,0x03,0x04,0x05};
uint8_t cnt=0;                   //¿ØÖÆÆµÂÊÑ­»·±äÁ¿
uint8_t control_period=6;        //¿ØÖÆÆµÂÊ=control_period*10ms
uint8_t set_pwm_period=1;        
uint8_t init_flags=1;

//Ê¹ÓÃmpu6050Ê±ÓÃµ½µÄ±äÁ¿
short Accel[3];                  //¼ÇÂ¼¼ÓËÙ¶ÈÔ­Ê¼Êı¾İ
float Accel_g[3];                //¼ÇÂ¼¼ÓËÙ¶ÈÒÔgÎªµ¥Î»Ê±µÄÊı¾İ
short Gyro[3];                   //¼ÇÂ¼½ÇËÙ¶ÈÔ­Ê¼Êı¾İ
long Quat[4];                    //¼ÇÂ¼ËÄÔªÊıÔ­Ê¼Êı¾İ
float Omiga[3];                  //¼ÇÂ¼½ÇËÙ¶ÈÒÔ»¡¶ÈÎªµ¥Î»µÄÊı¾İ£¨²âÊÔÊ±Ê¹ÓÃ£©
short Temp;
unsigned long sensor_timestamp;  //´«¸ĞÆ÷Ê±¼ä´Á
unsigned char more;              //
short sensors;
float gyro;                      //¼ÇÂ¼½ÇËÙ¶ÈÒÔ»¡¶ÈÎªµ¥Î»µÄÊı¾İ£¨Õı³£¹¤×÷Ê±Ê¹ÓÃ£©
float gyro_zero=0;                 //¼ÇÂ¼ÁãµãÆ¯ÒÆ
uint8_t gyro_num = 200;                //ÁãµãÆ¯ÒÆÊıÁ¿

int result;                      //³õÊ¼»¯½á¹û
struct int_param_s int_param_s;  //³õÊ¼»¯½á¹¹Ìå

//¿¨¶ûÂüÂË²¨Ê¹ÓÃµ½µÄ±äÁ¿

kalman1_state kalman_state;      //¿¨¶ûÂü½á¹¹Ìå
float kalman_x=1.0;
float kalman_p=1.0;
float kalman_result=0.0;



/* Variables Used in main.c File End*****************/


/* Variables Used in bsp_contral.c Files Begin *****/
double pid_lpf=0.6;     //pidÊä³öÂË²¨ÏµÊı


/* Variables Used in bsp_move.c File Begin **********/
//ps2Ò£à ˜ÔƒÕ½Ö„Ò¤
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


