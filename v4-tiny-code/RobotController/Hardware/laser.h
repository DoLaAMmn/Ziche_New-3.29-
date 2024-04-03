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
	uint32_t distance;  						/* 距离数据：测量目标距离单位 mm */
	uint16_t noise;		 						/* 环境噪声：当前测量环境下的外部环境噪声，越大说明噪声越大 */
	uint32_t peak;								/* 接收强度信息：测量目标反射回的光强度 */
	uint8_t confidence;						/* 置信度：由环境噪声和接收强度信息融合后的测量点的可信度 */
	uint32_t intg;     						/* 积分次数：当前传感器测量的积分次数 */
	int16_t reftof;   						/* 温度表征值：测量芯片内部温度变化表征值，只是一个温度变化量无法与真实温度对应 */
}LidarPointTypedef;

extern LidarPointTypedef Pack_Data[4][12];		/* 雷达接收的数据储存在这个变量之中 */
extern LidarPointTypedef Pack_sum[4];     /* 输出结果储存 */


#define HEADER 0xAA							/* 起始符 */
#define device_address 0x00     /* 设备地址 */
#define chunk_offset 0x00       /* 偏移地址命令 */
#define PACK_GET_DISTANCE 0x02 	/* 获取测量数据命令 */
#define PACK_RESET_SYSTEM 0x0D 	/* 复位命令 */
#define PACK_STOP 0x0F 				  /* 停止测量数据传输命令 */
#define PACK_ACK 0x10           /* 应答码命令 */
#define PACK_VERSION 0x14       /* 获取传感器信息命令 */




void data_process(uint8_t j);
void get_pose(uint16_t new_laser_xl,uint16_t new_laser_xr,uint16_t new_laser_yu,uint16_t new_laser_yd);
void get_data(uint8_t temp_data[400],uint8_t i);

#endif