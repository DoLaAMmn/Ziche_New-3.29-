#ifndef __MAP_H__
#define __MAP_H__

#include "stm32f4xx_hal.h"
#include "delay.h"
#include "motor_controller.h"

#define PULSE_N  390     // ����תһȦ��������������													  
#define DIAMETER  54     // ����ֱ������λmm
#define AXLE_L   210
// �᳤����λmm

#define LENGTH    380    // ������������֮��ľ���
#define STILLGO   40 	 // ����ת��ָ�������ߵľ���

#define S_SPEED  300	 // ֱ���ٶ�
#define T_SPEED  200	 // ת���ٶ�
#define B_SPEED  200     // �����ٶ�

#define PID_KP 3         // PID����-KP
#define PID_KI 0         // PID����-KI
#define PID_KD 3         // PID����-KD
#define FILTERPERCENT 1  // һ�׵�ͨ�˲�ϵ��
#define TRACK_TARGET 72  // Ѱ��Ŀ��λ��
typedef struct 
{
	uint16_t loc_xl;
	uint16_t loc_xr;
	uint16_t loc_yu;
	uint16_t loc_yd;
	
}MAP;

extern MAP pos[20];

void set_pos(uint16_t x,uint16_t y,uint8_t i);

void Stop(void);
void Left(int16_t n,int32_t T_Speed);                //ֱ�е�ָ��·����ת
void Right(int16_t n,int32_t T_Speed);               //ֱ�е�ָ��·����ת
void Straight(int16_t n,int32_t S_Speed);			 //ֱ�е�ָ��·��
void Back(int16_t n,int32_t T_Speed);                 //ֱ�е�ָ��·�ڵ�ͷ
int32_t Track_PID(int16_t position);			    // PID��������ƫ��:positionΪ��ǰλ

void zuobiao(void);


void zuozhuan(void); 
void youzhuan(void);
void zuobiao(void);
#endif
