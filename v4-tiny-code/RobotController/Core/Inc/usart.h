/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart5;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */


#define FRAME_BYTE_LENGTH 9 //串口通讯一帧数据的字节数（含帧头和帧尾），譬如20个字节为一个完整的数据帧，第1个字节帧头，第2个字节代表命令类型，第3~6字节是命令参数，第7个字节为帧尾
#define FRAME_START 0xA5 //帧头
#define FRAME_END 0x5A  //帧尾


typedef struct
{
	char RxBuffer[FRAME_BYTE_LENGTH];   //接收缓冲区
	uint8_t aRxBuffer;			//接收中断缓冲
	uint8_t Rx_Cnt; 		//接收缓冲计数
	uint8_t USART_FrameFlag;//接收完整数据帧标志，1完整，0不完整
} UartStruct;


extern UartStruct uart2Data;  //usart3的数据结构体

extern uint8_t IsUart1;  //串口1中断标志
extern uint8_t IsUart5;
extern uint8_t IsUart3;
extern uint8_t IsUart6;


void USART_GetChar(UartStruct *Uartn,uint8_t nChar); //串口接收到一个字节


extern uint8_t Uart1_Receive_buf[400];          //串口1接收中断数据存放的缓冲区
extern uint8_t Uart3_Receive_buf[400];          //串口3接收中断数据存放的缓冲区
extern uint8_t Uart5_Receive_buf[400];          //串口5接收中断数据存放的缓冲区
extern uint8_t Uart6_Receive_buf[400];          //串口6接收中断数据存放的缓冲区


extern uint16_t receive_cnt[];
extern uint16_t distance_X1;
extern uint16_t distance_Y1;
extern uint16_t distance_X2;
extern uint16_t distance_Y2;



/* USER CODE END Private defines */

void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

