#include "laser.h"
#include "stdio.h"
#include "math.h"
#include "usart.h"
#include "stdlib.h"

uint8_t pix_cm = 2;
uint16_t Pose[] = {1200, 400};
uint16_t map_size[] = {3200, 4000};
uint16_t car_size[] = {300, 300};
uint16_t pre_laser_xl = 400;
uint16_t pre_laser_xr = 2800;
uint16_t pre_laser_yu = 3850;
uint16_t pre_laser_yd = 150;

int16_t difference;
static uint8_t state = 0;	   // 状态位
static uint8_t cnt = 0;		   // 用于一帧12个点的计数
static uint8_t crc = 0;		   // 校验和
static uint8_t PACK_FLAG = 0;  // 命令标志位
static uint8_t data_len = 0;   // 数据长度
static uint32_t timestamp = 0; // 时间戳
static uint8_t state_flag = 1; // 转入数据接收标志位
static uint8_t state_num;

LidarPointTypedef Pack_Data[4][12]; /* 雷达接收的数据储存在这个变量之中 */
LidarPointTypedef Pack_sum[4];		/* 输出结果储存 */

// void get_pose(uint16_t new_laser_xl,uint16_t new_laser_xr,uint16_t new_laser_yu,uint16_t new_laser_yd)
//{
//	if (abs(new_laser_xl+new_laser_xr-map_size[0])<3*pix_cm+car_size[0])
//	{
//		Pose[0]=new_laser_xl+car_size[0]/2;
//	}
//	if (abs(new_laser_yu+new_laser_yd-map_size[1])<3*pix_cm+car_size[1])
//	{
//		Pose[1]=new_laser_yd+car_size[0]/2;
//	}
//	else
//	{
//		if(abs(new_laser_xl+new_laser_xr-pre_laser_xl-pre_laser_xr)<20*pix_cm)
//		{
//			difference=new_laser_xl-new_laser_xr-pre_laser_xl+pre_laser_xr;
//			Pose[0]=Pose[0]+difference/2;
//		}
//		if(abs(new_laser_yu+new_laser_yd-pre_laser_yu-pre_laser_yd)<20*pix_cm)
//		{
//			difference=new_laser_yu-new_laser_yd-pre_laser_yu+pre_laser_yd;
//			Pose[1]=Pose[1]+difference/2;
//		}
//	}
//	pre_laser_xl=new_laser_xl;
//	pre_laser_xr=new_laser_xr;
//	pre_laser_yu=new_laser_yu;
//	pre_laser_yd=new_laser_yd;
//
// }

void data_process(uint8_t j) /*数据处理函数，完成一帧之后可进行数据处理*/
{
	/* 计算距离 */
	uint8_t i;
	uint8_t count = 0;

	for (i = 0; i < 12; i++) /* 12个点取平均 */
	{
		if (Pack_Data[j][i].distance != 0) /* 去除0的点 */
		{
			count++;
			Pack_sum[j].distance += Pack_Data[j][i].distance;
			Pack_sum[j].noise += Pack_Data[j][i].noise;
			Pack_sum[j].peak += Pack_Data[j][i].peak;
			Pack_sum[j].confidence += Pack_Data[j][i].confidence;
			Pack_sum[j].intg += Pack_Data[j][i].intg;
			Pack_sum[j].reftof += Pack_Data[j][i].reftof;
		}
	}
	switch (j)
	{
	case 0:
		distance_X1 = Pack_sum[j].distance / count;
		break;
	case 1:
		distance_Y1 = Pack_sum[j].distance / count;
		break;
	case 2:
		distance_Y2 = Pack_sum[j].distance / count;
		break;
	case 3:
		distance_X2 = Pack_sum[j].distance / count;
		break;
	}
//	if (count != 0)
//	{
//		if (IsUart1)
//			distance_X1 = Pack_sum[j].distance / count;
//		else if (IsUart5)
//			distance_Y1 = Pack_sum[j].distance / count;
//		else if (IsUart3)
//			distance_Y2 = Pack_sum[j].distance / count;
//		else if (IsUart6)
//			distance_X2 = Pack_sum[j].distance / count;

		Pack_sum[j].distance = 0;
		Pack_sum[j].noise = 0;
		Pack_sum[j].peak = 0;
		Pack_sum[j].confidence = 0;
		Pack_sum[j].intg = 0;
		Pack_sum[j].reftof = 0;
		count = 0;
//	}
}

void get_pose(uint16_t new_laser_xl, uint16_t new_laser_xr, uint16_t new_laser_yu, uint16_t new_laser_yd)
{
	if (abs(new_laser_xl + new_laser_xr - map_size[0]) < 3 * pix_cm + car_size[0])
	{
		Pose[0] = new_laser_xl + car_size[0] / 2;
	}
	if (abs(new_laser_yu + new_laser_yd - map_size[1]) < 3 * pix_cm + car_size[1])
	{
		Pose[1] = new_laser_yd + car_size[0] / 2;
	}
	else
	{
//		if (abs(new_laser_xl + new_laser_xr - map_size[0]) > 100 * pix_cm + car_size[0])
//		{
			if (abs(new_laser_xl + new_laser_xr - pre_laser_xl - pre_laser_xr) < 35 * pix_cm)
			{
				difference = new_laser_xl - new_laser_xr - pre_laser_xl + pre_laser_xr;
				Pose[0] = Pose[0] + difference / 2;
			}
			if (abs(new_laser_yu + new_laser_yd - pre_laser_yu - pre_laser_yd) < 35 * pix_cm)
			{
				difference = new_laser_yd - new_laser_yu - pre_laser_yd + pre_laser_yu;
				Pose[1] = Pose[1] + difference / 2;
			}
//		}

		//printf("X::%d,Y::%d\n", Pose[0], Pose[1]);
	}
	pre_laser_xl = new_laser_xl;
	pre_laser_xr = new_laser_xr;
	pre_laser_yu = new_laser_yu;
	pre_laser_yd = new_laser_yd;

	//printf("xl::%d,xr::%d,yu::%d,yd::%d\n", pre_laser_xl, pre_laser_xr, pre_laser_yu, pre_laser_yd);
}

void get_data(uint8_t temp_data[], uint8_t i)
{
	for (int j = 0; j < 400; j++)
	{
		if (state < 4) /* 起始符验证 前4个数据均为0xAA */
		{
			if (temp_data[j] == HEADER)
				state++;
			else
				state = 0;
		}
		else if (state < 10 && state > 3)
		{
			switch (state)
			{
			case 4:
				if (temp_data[j] == device_address) /* 设备地址验证 */
				{
					state++;
					crc = crc + temp_data[j];
					break;
				}
				else
					state = 0, crc = 0;
			case 5:
				if (temp_data[j] == PACK_GET_DISTANCE) /* 获取测量数据命令 */
				{
					PACK_FLAG = PACK_GET_DISTANCE;
					state++;
					crc = crc + temp_data[j];
					break;
				}

				else if (temp_data[j] == PACK_RESET_SYSTEM) /* 复位命令 */
				{
					PACK_FLAG = PACK_RESET_SYSTEM;
					state++;
					crc = crc + temp_data[j];
					break;
				}
				else if (temp_data[j] == PACK_STOP) /* 停止测量数据传输命令 */
				{
					PACK_FLAG = PACK_STOP;
					state++;
					crc = crc + temp_data[j];
					break;
				}
				else if (temp_data[j] == PACK_ACK) /* 应答码命令 */
				{
					PACK_FLAG = PACK_ACK;
					state++;
					crc = crc + temp_data[j];
					break;
				}
				else if (temp_data[j] == PACK_VERSION) /* 获取传感器信息命令 */
				{
					PACK_FLAG = PACK_VERSION,
					state++,
					crc = crc + temp_data[j];
					break;
				}
				else
					state = 0, crc = 0;
			case 6:
				if (temp_data[j] == chunk_offset) /* 偏移地址 */
				{
					state++;
					crc = crc + temp_data[j];
					break;
				}
				else
					state = 0, crc = 0;
			case 7:
				if (temp_data[j] == chunk_offset)
				{
					state++;
					crc = crc + temp_data[j];
					break;
				}
				else
					state = 0, crc = 0;
			case 8:
				data_len = (uint16_t)temp_data[j]; /* 数据长度低八位 */
				state++;
				crc = crc + temp_data[j];
				break;
			case 9:
				data_len = data_len + ((uint16_t)temp_data[j] << 8); /* 数据长度高八位 */
				state++;
				crc = crc + temp_data[j];
				break;
			default:
				break;
			}
		}
		else if (state == 10)
			state_flag = 0;									   /*由switch跳出来时state为10，但temp_data[j]仍为距离长度高八位数据，需跳过一次中断*/
		if (PACK_FLAG == PACK_GET_DISTANCE && state_flag == 0) /* 获取一帧数据并校验 */
		{
			if (state > 9)
			{
				if (state < 190)
				{

					state_num = (state - 10) % 15;
					switch (state_num)
					{
					case 0:
						Pack_Data[i][cnt].distance = (uint16_t)temp_data[j]; /* 距离数据低八位 */
						crc = crc + temp_data[j];
						state++;
						break;
					case 1:
						Pack_Data[i][cnt].distance = ((uint16_t)temp_data[j] << 8) + Pack_Data[i][cnt].distance; /* 距离数据 */
						crc = crc + temp_data[j];
						state++;
						break;
					case 2:
						Pack_Data[i][cnt].noise = (uint16_t)temp_data[j]; /* 环境噪音低八位 */
						crc = crc + temp_data[j];
						state++;
						break;
					case 3:
						Pack_Data[i][cnt].noise = ((uint16_t)temp_data[j] << 8) + Pack_Data[i][cnt].noise; /* 环境噪音 */
						crc = crc + temp_data[j];
						state++;
						break;
					case 4:
						Pack_Data[i][cnt].peak = (uint32_t)temp_data[j]; /* 接受强度信息低八位 */
						crc = crc + temp_data[j];
						state++;
						break;
					case 5:
						Pack_Data[i][cnt].peak = ((uint32_t)temp_data[j] << 8) + Pack_Data[i][cnt].peak;
						crc = crc + temp_data[j];
						state++;
						break;
					case 6:
						Pack_Data[i][cnt].peak = ((uint32_t)temp_data[j] << 16) + Pack_Data[i][cnt].peak;
						crc = crc + temp_data[j];
						state++;
						break;
					case 7:
						Pack_Data[i][cnt].peak = ((uint32_t)temp_data[j] << 24) + Pack_Data[i][cnt].peak; /* 接受强度信息 */
						crc = crc + temp_data[j];
						state++;
						break;
					case 8:
						Pack_Data[i][cnt].confidence = temp_data[j]; /* 置信度 */
						crc = crc + temp_data[j];
						state++;
						break;
					case 9:
						Pack_Data[i][cnt].intg = (uint32_t)temp_data[j]; /* 积分次数低八位 */
						crc = crc + temp_data[j];
						state++;
						break;
					case 10:
						Pack_Data[i][cnt].intg = ((uint32_t)temp_data[j] << 8) + Pack_Data[i][cnt].intg;
						crc = crc + temp_data[j];
						state++;
						break;
					case 11:
						Pack_Data[i][cnt].intg = ((uint32_t)temp_data[j] << 16) + Pack_Data[i][cnt].intg;
						crc = crc + temp_data[j];
						state++;
						break;
					case 12:
						Pack_Data[i][cnt].intg = ((uint32_t)temp_data[j] << 24) + Pack_Data[i][cnt].intg; /* 积分次数 */
						crc = crc + temp_data[j];
						state++;
						break;
					case 13:
						Pack_Data[i][cnt].reftof = (int16_t)temp_data[j]; /* 温度表征值低八位 */
						crc = crc + temp_data[j];
						state++;
						break;
					case 14:
						Pack_Data[i][cnt].reftof = ((int16_t)temp_data[j] << 8) + Pack_Data[i][cnt].reftof; /* 温度表征值 */
						crc = crc + temp_data[j];
						state++;
						cnt++; /* 进入下一个测量点 */
						break;
					default:
						break;
					}
				}
				/* 时间戳 */
				if (state == 190)
					timestamp = temp_data[j], state++, crc = crc + temp_data[j];
				else if (state == 191)
					timestamp = ((uint32_t)temp_data[j] << 8) + timestamp, state++, crc = crc + temp_data[j];
				else if (state == 192)
					timestamp = ((uint32_t)temp_data[j] << 16) + timestamp, state++, crc = crc + temp_data[j];
				else if (state == 193)
					timestamp = ((uint32_t)temp_data[j] << 24) + timestamp, state++, crc = crc + temp_data[j];
				else if (state == 194)
				{

					data_process(i);  /* 数据处理函数，完成一帧之后可进行数据处理 */
					receive_cnt[i]++; /* 输出接收到正确数据的次数 */
					crc = 0;
					state = 0;
					state_flag = 1;
					cnt = 0; /* 复位*/
					break;
				}
			}
		}
	}
}
