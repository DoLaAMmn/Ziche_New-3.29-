#ifndef  __LASER_H__
#define  __LASER_H__

#include "stm32f4xx.h"
extern uint8_t pix_cm;
extern uint16_t Pose[];
extern uint16_t map_size[];
extern uint16_t car_size[];
extern uint16_t pre_laser_xl;
extern uint16_t pre_laser_xr;
extern uint16_t pre_laser_yu;
extern uint16_t pre_laser_yd;

typedef struct {
	uint32_t distance;  						/* �������ݣ�����Ŀ����뵥λ mm */
	uint16_t noise;		 						/* ������������ǰ���������µ��ⲿ����������Խ��˵������Խ�� */
	uint32_t peak;								/* ����ǿ����Ϣ������Ŀ�귴��صĹ�ǿ�� */
	uint8_t confidence;						/* ���Ŷȣ��ɻ��������ͽ���ǿ����Ϣ�ںϺ�Ĳ�����Ŀ��Ŷ� */
	uint32_t intg;     						/* ���ִ�������ǰ�����������Ļ��ִ��� */
	int16_t reftof;   						/* �¶ȱ���ֵ������оƬ�ڲ��¶ȱ仯����ֵ��ֻ��һ���¶ȱ仯���޷�����ʵ�¶ȶ�Ӧ */
}LidarPointTypedef;

extern LidarPointTypedef Pack_Data[4][12];		/* �״���յ����ݴ������������֮�� */
extern LidarPointTypedef Pack_sum[4];     /* ���������� */


#define HEADER 0xAA							/* ��ʼ�� */
#define device_address 0x00     /* �豸��ַ */
#define chunk_offset 0x00       /* ƫ�Ƶ�ַ���� */
#define PACK_GET_DISTANCE 0x02 	/* ��ȡ������������ */
#define PACK_RESET_SYSTEM 0x0D 	/* ��λ���� */
#define PACK_STOP 0x0F 				  /* ֹͣ�������ݴ������� */
#define PACK_ACK 0x10           /* Ӧ�������� */
#define PACK_VERSION 0x14       /* ��ȡ��������Ϣ���� */




void data_process(uint8_t j);
void get_pose(uint16_t new_laser_xl,uint16_t new_laser_xr,uint16_t new_laser_yu,uint16_t new_laser_yd);
void get_data(uint8_t temp_data[400],uint8_t i);

#endif