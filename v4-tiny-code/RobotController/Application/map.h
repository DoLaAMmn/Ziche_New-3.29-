#ifndef __MAP_H__
#define __MAP_H__

#include "stm32f4xx_hal.h"
#include "delay.h"
#include "motor_controller.h"

#define PULSE_N  390     // 轮子转一圈输出的脉冲个数；													  
#define DIAMETER  54     // 轮子直径，单位mm
#define AXLE_L   210
// 轴长，单位mm

#define LENGTH    380    // 相邻两条白线之间的距离
#define STILLGO   40 	 // 发出转弯指令后继续走的距离

#define S_SPEED  300	 // 直行速度
#define T_SPEED  200	 // 转弯速度
#define B_SPEED  200     // 后退速度

#define PID_KP 3         // PID参数-KP
#define PID_KI 0         // PID参数-KI
#define PID_KD 3         // PID参数-KD
#define FILTERPERCENT 1  // 一阶低通滤波系数
#define TRACK_TARGET 72  // 寻迹目标位置
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
void Left(int16_t n,int32_t T_Speed);                //直行到指定路口左转
void Right(int16_t n,int32_t T_Speed);               //直行到指定路口右转
void Straight(int16_t n,int32_t S_Speed);			 //直行到指定路口
void Back(int16_t n,int32_t T_Speed);                 //直行到指定路口掉头
int32_t Track_PID(int16_t position);			    // PID导出纠正偏差:position为当前位

void zuobiao(void);


void zuozhuan(void); 
void youzhuan(void);
void zuobiao(void);
#endif
