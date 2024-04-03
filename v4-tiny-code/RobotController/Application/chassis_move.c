#include "math.h"
#include "chassis_move.h"
#include "motor_controller.h"
#include "laser.h"
#include "usart.h"
#include "stdlib.h"
#include "map.h"
//使用本程序时切记如果底盘安装方式不同会导致解算出的值不同！！！！
//顺时针角度为正
Chassis chassis;
void chassis_move(float target_speed, float target_theta, float target_omega)//target_speed: m/s, target_dir: rad, target_omega: rad/s 角速度和角度值由imu提供
{
    chassis.Radius = 150; // 底盘半径，单位：mm
    chassis.angle+=0.01*target_omega;//底盘自旋情况下，改变机器人坐标系和全局坐标系的粗略计算
    float speed[4]; // 速度计算值

    float sin_ang = sin(chassis.angle); // 当前角度的sin值
    float cos_ang = cos(chassis.angle); // 当前角度的cos值

    // 全局坐标系下的x、y轴速度计算
    float speed_X = target_speed * cos(target_theta); // x轴速度，单位：m/s
    float speed_Y = target_speed * sin(target_theta); // y轴速度，单位：m/s

    // 底盘移动速度的四个分量计算
    speed[0] = -((-cos_ang - sin_ang) * speed_X + (-sin_ang + cos_ang) * speed_Y + chassis.Radius * target_omega) / sqrt(2);
    speed[1] = -((-cos_ang + sin_ang) * speed_X + (-sin_ang - cos_ang) * speed_Y + chassis.Radius * target_omega) / sqrt(2);
    speed[2] = -((cos_ang - sin_ang) * speed_X + (sin_ang + cos_ang) * speed_Y + chassis.Radius * target_omega) / sqrt(2);
    speed[3] = -((cos_ang + sin_ang) * speed_X + (sin_ang - cos_ang) * speed_Y + chassis.Radius * target_omega) / sqrt(2);

    for (int i = 0; i < 4; i++)
    {
        MotorController_SetSpeed(i + 1, (int)speed[i]); // 设置电机速度
    }
    // HAL_Delay(10);
}

float chassis_pid(float error)
{
	float kp=0.8;
	float ki=0.1;
	float kd=0;
	float last_error;
	float pre_error;
	float result;
	result=kp*(error-last_error)+ki*error+kd*(error-2*last_error+pre_error);
	pre_error=last_error;
	last_error=error;
	if(result>250)result=250;
	return result;
	
}
