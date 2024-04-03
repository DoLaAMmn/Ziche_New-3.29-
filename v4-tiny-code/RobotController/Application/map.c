#include "map.h"
#include "stm32f4xx_hal.h"
#include "delay.h"
#include "motor_controller.h"
#include "motor_driver.h"
#include "chassis_move.h"

//==========坐标系变量=============
uint8_t m=0;
int32_t x=0;
int32_t y=0;
MAP pos[20];


//===========运动函数=============
void set_pos(uint16_t x,uint16_t y,uint8_t i)
{
	pos[i].loc_xl=x;
	pos[i].loc_xr=3200-x;
	pos[i].loc_yd=y;
	pos[i].loc_yu=4000-y;
}
void Straight(int16_t n,int32_t S_Speed)	// 直行函数：n为设置的经过十字路口个数，S_Speed为直行速度
{

        chassis_move(S_Speed,0,0);
//        HAL_Delay(500);                                     //初代码用延时
//        Stop();
}
void Left(int16_t n,int32_t T_Speed)                        //左转指定路口
{

        chassis_move(T_Speed,3.14*3/2,0);
        HAL_Delay(500);
        Stop();

}
void Right(int16_t n,int32_t T_Speed)                        //右转指定路口
{
        chassis_move(T_Speed,3.14/2,0);
        HAL_Delay(500);
        Stop();
}
void Back(int16_t n,int32_t T_Speed)                        //右转指定路口
{
        chassis_move(T_Speed,3.14,0);
        HAL_Delay(500);
        Stop();

}

void Stop(void)
{
        chassis_move(0,0,0);
        HAL_Delay(500);
}

int32_t Track_PID(int16_t position)	// PID导出纠正偏差:position为当前位置（暂时没用，需要点激光后再改）
{
    static int32_t LastFilter = 0;	// 前次滤波值
	static int32_t Last_error = 0;	// 上一次的误差，用静态变量储存
	static int32_t Error_acc = 0;	// 累计误差，用静态变量储存
	int32_t iError = TRACK_TARGET - position;	// 当前误差
	int32_t Out_correct = 0;		// 输出纠正数
	Error_acc = Error_acc + iError;	// 累计误差
	Out_correct = PID_KP*iError + PID_KI*Error_acc + PID_KD*(iError-Last_error);
	// PID = 比例系数*当前误差+积分系数*累计误差+微分系数*（当前误差-上一次误差）
	Last_error = iError;			// 储存误差
    Out_correct = FILTERPERCENT * Out_correct + (1 - FILTERPERCENT)*LastFilter;
    LastFilter = Out_correct;		// 滤波值存储
	return Out_correct;
}
//==========坐标系变换函数==================
void zuozhuan(void)
{
    if(m==0) m=3;
    else m=m-1;
}
void youzhuan(void)
{
    if(m==3) m=0;
    else m=m+1;
}
void zuobiao(void)
{
    if(m==0) y++;
    else if(m==1) x++;
    else if(m==3) x--;
    else y--;
}

