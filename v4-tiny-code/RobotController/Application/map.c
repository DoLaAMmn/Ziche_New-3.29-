#include "map.h"
#include "stm32f4xx_hal.h"
#include "delay.h"
#include "motor_controller.h"
#include "motor_driver.h"
#include "chassis_move.h"

//==========����ϵ����=============
uint8_t m=0;
int32_t x=0;
int32_t y=0;
MAP pos[20];


//===========�˶�����=============
void set_pos(uint16_t x,uint16_t y,uint8_t i)
{
	pos[i].loc_xl=x;
	pos[i].loc_xr=3200-x;
	pos[i].loc_yd=y;
	pos[i].loc_yu=4000-y;
}
void Straight(int16_t n,int32_t S_Speed)	// ֱ�к�����nΪ���õľ���ʮ��·�ڸ�����S_SpeedΪֱ���ٶ�
{

        chassis_move(S_Speed,0,0);
//        HAL_Delay(500);                                     //����������ʱ
//        Stop();
}
void Left(int16_t n,int32_t T_Speed)                        //��תָ��·��
{

        chassis_move(T_Speed,3.14*3/2,0);
        HAL_Delay(500);
        Stop();

}
void Right(int16_t n,int32_t T_Speed)                        //��תָ��·��
{
        chassis_move(T_Speed,3.14/2,0);
        HAL_Delay(500);
        Stop();
}
void Back(int16_t n,int32_t T_Speed)                        //��תָ��·��
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

int32_t Track_PID(int16_t position)	// PID��������ƫ��:positionΪ��ǰλ�ã���ʱû�ã���Ҫ�㼤����ٸģ�
{
    static int32_t LastFilter = 0;	// ǰ���˲�ֵ
	static int32_t Last_error = 0;	// ��һ�ε����þ�̬��������
	static int32_t Error_acc = 0;	// �ۼ����þ�̬��������
	int32_t iError = TRACK_TARGET - position;	// ��ǰ���
	int32_t Out_correct = 0;		// ���������
	Error_acc = Error_acc + iError;	// �ۼ����
	Out_correct = PID_KP*iError + PID_KI*Error_acc + PID_KD*(iError-Last_error);
	// PID = ����ϵ��*��ǰ���+����ϵ��*�ۼ����+΢��ϵ��*����ǰ���-��һ����
	Last_error = iError;			// �������
    Out_correct = FILTERPERCENT * Out_correct + (1 - FILTERPERCENT)*LastFilter;
    LastFilter = Out_correct;		// �˲�ֵ�洢
	return Out_correct;
}
//==========����ϵ�任����==================
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

