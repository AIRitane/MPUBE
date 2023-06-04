#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include "stm32f4xx_hal.h"
#include "bsp_imu.h"
#include "send_task.h"

#define A 65
#define B 66
#define C 67
#define D 68
#define E 69
#define F 70

#define TASK3_ACC_BUFFER_LEN 100
#define TASK3_AVER_ACC_DROPOUT 500
#define TASK3_AVER_ACC_NUM 100

#define TASK3_ACCY_MAX 2000
#define TASK3_ACCY_POINT_MAX 8
#define TASK3_ACCY_MIN -2000
#define TASK3_ACCY_POINT_MIN 8
#define TASK3_ACCX_MAX 2000
#define TASK3_ACCX_POINT_MAX 8
#define TASK3_ACCX_MIN -2000
#define TASK3_ACCX_POINT_MIN 8

typedef enum
{
	no_mode,
	mode1,
	mode2,
	mode3,
	stop_mode
} mode_e;

typedef struct
{
	float pitch;
	float roll;
	int top;
	mode_e mode;

	float ax, ay; // 滤波后的加速度
	uint8_t acc_is_refresh;

	int move_x[60], move_y[60];
	uint8_t move_len;
} cube_t;

typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
} imu_t;

extern cube_t cube;

void HC08_SendString_mode(uint8_t *strTemp, uint32_t len);
void Data_package_mode(uint8_t type, uint8_t data);
#endif