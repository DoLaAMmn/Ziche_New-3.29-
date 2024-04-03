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


#define FRAME_BYTE_LENGTH 9 //����ͨѶһ֡���ݵ��ֽ�������֡ͷ��֡β����Ʃ��20���ֽ�Ϊһ������������֡����1���ֽ�֡ͷ����2���ֽڴ����������ͣ���3~6�ֽ��������������7���ֽ�Ϊ֡β
#define FRAME_START 0xA5 //֡ͷ
#define FRAME_END 0x5A  //֡β


typedef struct
{
	char RxBuffer[FRAME_BYTE_LENGTH];   //���ջ�����
	uint8_t aRxBuffer;			//�����жϻ���
	uint8_t Rx_Cnt; 		//���ջ������
	uint8_t USART_FrameFlag;//������������֡��־��1������0������
} UartStruct;


extern UartStruct uart2Data;  //usart3�����ݽṹ��

extern uint8_t IsUart1;  //����1�жϱ�־
extern uint8_t IsUart5;
extern uint8_t IsUart3;
extern uint8_t IsUart6;


void USART_GetChar(UartStruct *Uartn,uint8_t nChar); //���ڽ��յ�һ���ֽ�


extern uint8_t Uart1_Receive_buf[400];          //����1�����ж����ݴ�ŵĻ�����
extern uint8_t Uart3_Receive_buf[400];          //����3�����ж����ݴ�ŵĻ�����
extern uint8_t Uart5_Receive_buf[400];          //����5�����ж����ݴ�ŵĻ�����
extern uint8_t Uart6_Receive_buf[400];          //����6�����ж����ݴ�ŵĻ�����


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

