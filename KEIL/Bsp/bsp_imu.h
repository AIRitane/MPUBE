#ifndef _IMU_H
#define _IMU_H

#include "stm32f4xx_hal.h"

typedef struct
{
	// 大地坐标系
	float __Pitch;
	float __Roll;
	float __Yaw;

	float Pitch_v;
	float Roll_v;
	float Yaw_v;

	// imu坐标系
	float ax;
	float ay;
	float az;

	float Pitch;
	float Roll;
	float Yaw;

	float Pitch_offset;
	float Roll_offset;
	float Yaw_offset;
	

	uint8_t data_refresh;
	uint8_t is_refresh;
	uint8_t is_shake;

} IMU_Info;

IMU_Info *IMU_GetInfo(void);
uint8_t IMU_Init(void);
void IMU_Update(void);
void IMU_Set_Is_Refresh(uint8_t status);
void IMU_Set_Data_Refresh(uint8_t status);
void IMU_Set_Is_Shake(uint8_t status);
#endif
