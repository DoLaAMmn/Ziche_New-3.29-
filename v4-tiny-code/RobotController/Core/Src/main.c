/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_driver.h"
#include "motor_controller.h"
#include "led.h"
#include "keys.h"
#include "mpu6500dmp.h"
#include "chassis_move.h"
#include "map.h"
#include "laser.h"
#include "math.h"
#include "stdlib.h"
#include "valuepack.h"
#include "pid_controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
    //鍙傛暟
   int32_t map[10][10]={0};
	 float theta;
	 float pid_add;
//===============缂栫爜鍣ㄦ祴璇曞彉閲?===============
volatile int32_t enc1, enc2, enc3, enc4;
volatile int32_t test_index;
//====================鐢垫満鏁呴殰銆佺數娴佹祴璇?==============

uint32_t motor_currents[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  MPU6500_DMP_Init(); // MPU6500鍒濆鍖?
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //===================鐢垫満娴嬭瘯===============
  MotorDriver_Init();
  MotorDriver_Start(4, MOTOR_PWM_DUTY_LIMIT / 2);
  MotorDriver_Start(3, MOTOR_PWM_DUTY_LIMIT / 2);
  MotorDriver_Start(2, MOTOR_PWM_DUTY_LIMIT / 2);
  MotorDriver_Start(1, MOTOR_PWM_DUTY_LIMIT / 2);

  Encoder_Init();
  //==================鐢垫満杞?熸帶鍒跺櫒鍚姩===============
  MotorController_Init();                // 鍒濆鍖栬皟閫熷櫒
  MotorController_Enable(ENABLE);


  //===================Usart3閫氫俊娴嬭瘯===============
  // HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer, 1);

  //=================led娴嬭瘯=================
  FnLED_SetRGB(FnLED2, 33, 0, 0, 1);
  uint8_t led_val = 0;
  HAL_Delay(500);
  FnLED_OFF(FnLED2);

  //=================鍔熻兘娴嬭瘯=================
  //chassis_move(200,1.57,0);
//  while(1){
//    Straight(1,700); //鐩磋
//    Back(1,700); // 鎺夊ご
//  }
	
	
	set_pos(1300,600,0);
	set_pos(1200,1200,1);
	set_pos(1000,1400,2);
	set_pos(1600,400,3);

    
//  Left(1,T_SPEED);    // 宸﹁浆
//  Right(1,T_SPEED); // 鍙宠浆
//  Back(1,T_SPEED); // 鎺夊ご
  //=================鍙傛暟=====================
  //鍦板浘
//  map[0][1]=2;
//  map[1][1]=1;
//  map[1][2]=1;
//  extern uint8_t m;
//  extern int32_t x;
//  extern int32_t y;
  
  
  while (1)
  {

		get_pose(distance_X1,distance_X2,distance_Y1,distance_Y2);
		// 定义一个足够大的数组用于存放数据包
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
unsigned char buffer[50];
// 开始发送
	startValuePack(buffer);
		// 2. 向数据包中放入各类数据，由于数据包的顺序结构是固定的，因此注意严格以如下的顺序进行存放，否则会出现错误

		putShort(distance_X1);
		putShort(distance_X2);
		putShort(distance_Y1);
		putShort(distance_Y2);
		putShort(Pose[0]);
		putShort(Pose[1]);
		putFloat(theta);
		// 通过串口发送，endValuePack返回了数据包的总长度
			sendBuffer(buffer,endValuePack());
//      extern double mpu_pitch,mpu_roll,mpu_yaw;
      //Straight(1,100); //鐩磋
//			     for (int i = 0; i < 4; i++)
//    {
//        MotorController_SetSpeed(i + 1, 70); // 设置电机速度
//    }
//    switch(map[x][y])
//     {
//    case 0:  
//        Straight(1,S_SPEED); //鐩磋
//        zuobiao();
//        break;
//   case 1:                  //宸﹁浆锛屽乏杞畬鎴愬悗缁х画鐩磋
//        Left(2,T_SPEED);    // 宸﹁浆
//        Straight(1,600);    //鐩磋
//        zuozhuan();
//        zuobiao();
//        break;
//     case 2:
//        Right(2,T_SPEED); // 鍙宠浆
//        Straight(1,600);    //鐩磋
//        youzhuan();
//        zuobiao();
//        break;
//     case 3:
//        Back(T_SPEED,-1); // 鎺夊ご
//        Straight(1,600);    //鐩磋
//        zuozhuan();
//        zuozhuan();
//        zuobiao();
//        break;
//     case 4:
//        Stop();
//        break;
//     }

    Get_MPU6500_DMP_Data();
    
    //HAL_Delay(50);
    

//    //==================绋嬫帶寮?鍏抽儴鍒嗘祴璇?===========
    		if(Key_Pressed(1)==1)
				{
					for(int i=0;i<4;i++)
					{
						theta=atan2(pos[i].loc_xl-Pose[0],pos[i].loc_yd-Pose[1]);
						while(1)
						{
							theta=atan2(pos[i].loc_xl-Pose[0],pos[i].loc_yd-Pose[1]);
							pid_add=chassis_pid((float)(sqrt((pos[i].loc_xl-Pose[0])*(pos[i].loc_xl-Pose[0])+(pos[i].loc_yd-Pose[1])*(pos[i].loc_yd-Pose[1]))));
							chassis_move(50+pid_add,theta,0);
							if(abs(pos[i].loc_xl-Pose[0])<3*pix_cm&&abs(pos[i].loc_yd-Pose[1])<3*pix_cm)
							{
								chassis_move(0,0,0);
								break;
							}
								startValuePack(buffer);
							// 2. 向数据包中放入各类数据，由于数据包的顺序结构是固定的，因此注意严格以如下的顺序进行存放，否则会出现错误

							putShort(distance_X1);
							putShort(distance_X2);
							putShort(distance_Y1);
							putShort(distance_Y2);
							putShort(Pose[0]);
							putShort(Pose[1]);
							putFloat(theta);
							// 通过串口发送，endValuePack返回了数据包的总长度
							sendBuffer(buffer,endValuePack());
							
					 	 }
						HAL_Delay(3000);
						
					 }
				}
    //		HAL_GPIO_TogglePin(SWITCH1_GPIO_Port,SWITCH1_Pin);
    //		HAL_GPIO_TogglePin(SWITCH2_GPIO_Port,SWITCH2_Pin);
    //		HAL_GPIO_TogglePin(SWITCH3_GPIO_Port,SWITCH3_Pin);
				

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
