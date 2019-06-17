
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "includes.h"
//#include "bsp_fitler.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint32_t tick;

double_u8_union d_u8;
//extern int_u8_union rx_spd;              //message used for speed control, receive from uart, unused in ps2 mode.
//double_u8_union tx_linear,tx_angular;
extern tx_message tx_msg;                  //message transmit with uart to PC to describe the motion state
extern uint8_t rx_ctrl_msg[4];             //command used to control the motion, used in UART interrupt, see BSP_Compare_Msg function (bsp_move.c)
extern uint8_t debug_flag;                 //flag to control printf

extern moto_pid bsp_pid[2];                //pid structure

extern double move_target_spd;             //target speed for forward
extern double turn_target_spd;             //target speed for turn

extern float set_spd[2];                   //change this to control speed directly, used in bsp_move.h
extern double set_spd_ps2[2];
extern double set_spd_pc[2];
extern double last_encoder[2];             //record last encoder
extern double now_encoder[2];					     //record current encoder for fitler
extern double current_spd[2];              //raw data for current speed
extern double last_spd[2];                 //
extern double speed_lpf[2];                //speed after fitler


extern float lpf_index;                    //index for lpf，less for smoother but slower,  lpf = 0.25

extern uint8_t msg[6];
extern uint8_t cnt;                        //frequency  T = cnt*1ms
extern uint8_t control_period;
extern uint8_t ps2_cnt;
extern uint8_t buff[38];

extern short Accel[3];
extern float Accel_g[3];
extern short Gyro[3];
extern long Quat[4];
extern float Omiga[3];
extern short Temp;
extern unsigned long sensor_timestamp;
extern unsigned char more;
extern short sensors;
extern float gyro;
extern float gyro_zero;                 //记录零点漂移
extern uint8_t gyro_num;                //零点漂移数量

extern int result;
extern struct int_param_s int_param_s;

extern kalman1_state kalman_state;
extern float kalman_x;
extern float kalman_p;
extern float kalman_result;

extern uint16_t ps2_LX,ps2_LY,ps2_RX,ps2_RY,ps2_KEY;

extern navi_message navi_msg;
extern uint8_t lock_status;

extern control_mode mode;

/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	PS2_SetInit();
	//PS2_stdlib_Init();
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_Delay(5);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_Delay(5);
	
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);//PWM输出
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);//PWM输出
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);//PWM输出
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);//PWM输出
	
	//PID init
	BSP_Pid_Init(&bsp_pid[0],pid_id_1,5.0,1,1.1);   //5 1  1.1
	BSP_Pid_Init(&bsp_pid[1],pid_id_2,5.0,1,1.1);   //5 1  1.1
	//p = 0.3,i = 0.08 at7:30
	
//	BSP_MPU6050_Init();
//	if (BSP_MPU6050_ReadID() == 0)
//	{	
//    //printf("cannot find MPU6050,stoooooop.\n");
//	  while(1);
//  }
	
	HAL_Delay(10);
	//HAL_UART_Receive_IT(&huart1,(uint8_t *)&rx_ctrl_msg[0],4);
	
//	BSP_MPU6050_Test();
//	
//	HAL_Delay(5000);
//	uint8_t i=0;
//	while(i<gyro_num)
//	{
//		i++;
//		dmp_read_fifo(Gyro,Accel,Quat,&sensor_timestamp,&sensors,&more);
//		gyro_zero += Gyro[2]*1.0/32.8;
//		HAL_Delay(1);
//	}
//	gyro_zero /= (1.0 * gyro_num);
	
	
	//kalman filter initilize
	//kalman1_init(&kalman_state,kalman_x,kalman_p);
	//PS2_SetInit();
	HAL_Delay(5000);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_Delay(100);
	//HAL_UART_Receive_IT(&huart1,(uint8_t *)&d_u8,sizeof(d_u8));
	printf("working...\n");
	for(int i = 0;i<5;i++)
	{
		HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin);
		HAL_Delay(200);
	}
	
	tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(HAL_GetTick() - tick >= 50)      //50ms loop
		{
			//dmp_read_fifo(Gyro,Accel,Quat,&sensor_timestamp,&sensors,&more);
			tick = HAL_GetTick();
			//gyro = Gyro[2]*1.0/32.8 - gyro_zero;
			//kalman_result = kalman1_filter(&kalman_state,gyro);
			
			tx_msg.head = 0x01;
			//tx_msg.angular.data_float = gyro;       //使用mpu6050计算角速度
			tx_msg.angular.data_float = (float)(speed_lpf[moto_ID_1]-speed_lpf[moto_ID_2])/13.0/51.0*3.1416*125/1000/0.5/2.5;   //使用编码器计算角速度 w = (v1-v2)/d
			
			//tx_msg.angular.data_float = (float)(current_spd[moto_ID_1])/13.0/51.0/2.0*125*3.142/180;
			tx_msg.linear.data_float = (float)(speed_lpf[moto_ID_1]+speed_lpf[moto_ID_2])/13.0/51.0/2.0* 3.1416*(125.0/1000)/2;   // v = n*pi*d  uint: m/s
			BSP_UART_DataLimit(&tx_msg);
			BSP_UART_Transmit(&tx_msg);
			
			//BSP_UART_Transmit(&tx_msg);

			//float rmp_1 = speed_lpf[moto_ID_1]*1.0/13/51;
			//float rmp_2 = speed_lpf[moto_ID_2]*1.0/13/51;
			
			printf("/////////////////////////////////////////////////////////\n");
			printf("ang :%.2f,line:%.2f\n",tx_msg.angular.data_float,tx_msg.linear.data_float);
			printf("spd1:%.2f,pid1:%.2f\n",speed_lpf[moto_ID_1],bsp_pid[moto_ID_1].out);
			printf("spd2:%.2f,pid2:%.2f\n",speed_lpf[moto_ID_2],bsp_pid[moto_ID_2].out);
			printf("set1:%.2f,set2:%.2f\n",set_spd[moto_ID_1],set_spd[moto_ID_2]);
			printf("/////////////////////////////////////////////////////////\n\n");
			
			//HAL_Delay(50);
		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		//printf("tim1 interupt!!!!\n");
		
		BSP_Pid_SetTarget(&bsp_pid[0],set_spd[0]);
		BSP_Pid_SetTarget(&bsp_pid[1],set_spd[1]);
		//HAL_UART_Transmit(&huart1,&msg[2],2,50);
		
		if(cnt==control_period)
		{
			
			/****************************/
			//当前编码器数值的变化
			now_encoder[moto_ID_1] = (BSP_Moto_GetValue(moto_ID_1)-last_encoder[moto_ID_1])*(100 / cnt);
			now_encoder[moto_ID_2] = (BSP_Moto_GetValue(moto_ID_2)-last_encoder[moto_ID_2])*(100 / cnt);
			//滤去 越过65536时编码器的数据
			if(now_encoder[moto_ID_1] > -20000 && now_encoder[moto_ID_1] < 20000) 
					current_spd[moto_ID_1] = now_encoder[moto_ID_1];
			if(now_encoder[moto_ID_2] > -20000 && now_encoder[moto_ID_2] < 20000) 
					current_spd[moto_ID_2] = now_encoder[moto_ID_2];
	
			//编码器速度滤波
			speed_lpf[moto_ID_1] = lpf_index * current_spd[moto_ID_1] + (1-lpf_index) * last_spd[moto_ID_1];
			speed_lpf[moto_ID_2] = lpf_index * current_spd[moto_ID_2] + (1-lpf_index) * last_spd[moto_ID_2];
			last_spd[moto_ID_1] =  speed_lpf[moto_ID_1];
			last_spd[moto_ID_2] =  speed_lpf[moto_ID_2];
			//记录 当前编码器数捿 作为 下次读取上次数据
			last_encoder[moto_ID_1] = BSP_Moto_GetValue(moto_ID_1);
			last_encoder[moto_ID_2] = BSP_Moto_GetValue(moto_ID_2);
			
			//将滤波后的速度传入pid
			BSP_Pid_SetCurrent(&bsp_pid[moto_ID_1],speed_lpf[moto_ID_1]);
			BSP_Pid_SetCurrent(&bsp_pid[moto_ID_2],speed_lpf[moto_ID_2]);
			//pid计算
			BSP_Pid_calculate(&bsp_pid[moto_ID_1]); 
			BSP_Pid_calculate(&bsp_pid[moto_ID_2]);
			cnt = 0;
			//BSP_Mode_Control();
			if(mode == control_mode_ps2)
			{
				HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_RESET);
				BSP_Move_Ps2_Scan_Command();
				BSP_Move_Ps2_Control();
			}
			else
			{
				HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_SET);
				HAL_UART_Receive_IT(&huart1,(uint8_t *)&buff,sizeof(buff));
			}
			
		}
		
		
		BSP_Moto_SpeedControlOutput(&bsp_pid[moto_ID_1],cnt);
		BSP_Moto_SpeedControlOutput(&bsp_pid[moto_ID_2],cnt);
		BSP_Moto_Control(bsp_pid[moto_ID_1].out,bsp_pid[moto_ID_2].out);
		
		//BSP_Moto_Control(300,00);
		cnt++;
		//HAL_UART_Receive_IT(&huart1,(uint8_t *)&navi_msg,sizeof(navi_msg));
	}
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		//printf("%d.\n",rx_spd.spd_int);
		//printf("UART_RECEIVE_IT\n");
		//BSP_NAVI_Control();
//		set_spd[0] = 300;
//		set_spd[1] = 300;
		BSP_UART_Receive();
		//printf("%lf...%lf...\n",navi_msg.linear.data_double,navi_msg.angular.data_double);
		BSP_NAVI_Control();
		HAL_UART_Receive_IT(&huart1,(uint8_t *)&buff,sizeof(buff));
		
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == MODE_SWITCH_Pin)
	{
		BSP_MODE_Switch();
		//printf("mode changed to %d.\n",mode);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
