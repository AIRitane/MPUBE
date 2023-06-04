#include "cmsis_os.h"
#include "bsp_imu.h"
#include "tim.h"
#include "usart.h"
#include "stdio.h"
#include "math.h"
#include "motion_task.h"

#define BUFFER_LEN 100
#define BUFFER_DELTA 10
#define SHAKE_THR 300000000
#define SHAKE_BUFFER_LEN BUFFER_LEN
#define DEBUG_USART

float imu_angle_buffer[3][BUFFER_LEN] = {900};
float imu_acc_buffer[3][SHAKE_BUFFER_LEN] = {0};

float FABS(float x)
{
	x = x > 0 ? x : -x;
	return x;
}

// 调试接口
#ifdef DEBUG_USART
#define DEBUG_PRE 5
uint32_t debug_peri = 0;

int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);
	return ch;
}

/*fgetc*/
int fgetc(FILE *f)
{
	int ch;
	while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == RESET)
		;
	HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, 1000);
	return (ch);
}
#endif

IMU_Info *imu_info;
float pitch_delta = 0, yaw_delta = 0, roll_delta = 0;
float acc_delta = 0;

void mpu_task(void const *argument)
{
	static uint32_t index = 0;
	float imu_angle_max[3] = {-1000};
	float imu_angle_min[3] = {1000};
	float imu_angle_temp[3] = {1000};

	IMU_Init();

	// 定频1000Hz解算
	HAL_TIM_Base_Start_IT(&htim2);

	// 亮灯管脚初始化
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOF_CLK_ENABLE();
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	imu_info = IMU_GetInfo();

	// 计算坐标系偏差量
	uint32_t waite_count = 0;
	for (;;)
	{
		if (imu_info->data_refresh)
		{
			IMU_Set_Data_Refresh(0);

			// 均值30数据
			if (waite_count < 30)
			{
				pitch_delta += imu_info->__Pitch;
				yaw_delta += imu_info->__Yaw;
				roll_delta += imu_info->__Roll;
			}
			else
			{
				pitch_delta /= 30;
				yaw_delta /= 30;
				roll_delta /= 30;
				break;
			}
			waite_count++;
		}
	}

	while (1)
	{
		float aver_acc[3] = {0};
		acc_delta = 0;

		if (imu_info->data_refresh)
		{
			IMU_Set_Data_Refresh(0);
			imu_angle_buffer[0][index] = imu_info->__Pitch;
			imu_angle_buffer[1][index] = imu_info->__Yaw;
			imu_angle_buffer[2][index] = imu_info->__Roll;
			imu_acc_buffer[0][index] = imu_info->ax;
			imu_acc_buffer[1][index] = imu_info->ay;
			imu_acc_buffer[2][index] = imu_info->az;

			index = (index + 1) % BUFFER_LEN;

			for (uint8_t j = 0; j < 3; j++)
			{
				imu_angle_min[j] = 1000;
				imu_angle_max[j] = -1000;
			}

			for (uint8_t i = 0; i < BUFFER_LEN; i++)
			{
				aver_acc[0] += imu_acc_buffer[0][i];
				aver_acc[1] += imu_acc_buffer[1][i];
				aver_acc[2] += imu_acc_buffer[2][i];

				for (uint8_t j = 0; j < 3; j++)
				{
					if (imu_angle_max[j] < imu_angle_buffer[j][i])
					{
						imu_angle_max[j] = imu_angle_buffer[j][i];
					}
					if (imu_angle_min[j] > imu_angle_buffer[j][i])
					{
						imu_angle_min[j] = imu_angle_buffer[j][i];
					}
				}
			}

			for (uint8_t j = 0; j < 3; j++)
			{
				aver_acc[j] /= BUFFER_LEN;
			}

			for (uint8_t i = 0; i < BUFFER_LEN; i++)
			{
				for (uint8_t j = 0; j < 3; j++)
				{
					acc_delta += (((imu_acc_buffer[j][i] - aver_acc[j]) * (imu_acc_buffer[j][i] - aver_acc[j])) / BUFFER_LEN);
				}
			}

			if (acc_delta > SHAKE_THR)
			{
				imu_info->is_shake = 1;
			}

			// ????????
			if (FABS(imu_angle_max[0] - imu_angle_min[0]) < BUFFER_DELTA && FABS(imu_angle_max[2] - imu_angle_min[2]) < BUFFER_DELTA)
			{
				for (uint8_t j = 0; j < 3; j++)
				{
					imu_angle_temp[j] = 0;
				}

				for (uint8_t i = 0; i < BUFFER_LEN; i++)
				{
					for (uint8_t j = 0; j < 3; j++)
					{
						imu_angle_temp[j] += imu_angle_buffer[j][i];
					}
				}

				imu_info->Pitch = imu_angle_temp[0] / BUFFER_LEN - pitch_delta;
				imu_info->Yaw = imu_angle_temp[1] / BUFFER_LEN - yaw_delta;
				imu_info->Roll = imu_angle_temp[2] / BUFFER_LEN - roll_delta;

				IMU_Set_Is_Refresh(1);
			}
		}

#ifdef DEBUG_USART
		if (debug_peri % DEBUG_PRE == 0)
		{
			debug_peri = 0;
			if (cube.acc_is_refresh)
			{
				//				cube.acc_is_refresh = 0;
				printf("%f,%f,%f,%f\n", cube.ax, cube.ay, imu_info->ax, imu_info->ay);
			}
		}

		debug_peri++;
#endif

		osDelay(1);
	}
}

// ?????
void set_mpu_base(void)
{
	while (imu_info->is_refresh == 0)
		;

	pitch_delta = imu_info->__Pitch;
	yaw_delta = imu_info->__Yaw;
	roll_delta = imu_info->__Roll;
}

uint32_t led_peri = 0;
void TIM2_IRQHandler(void)
{
	led_peri++;
	if (led_peri % 500 == 0)
	{
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
	}

	IMU_Update();
	IMU_Set_Data_Refresh(1);

	HAL_TIM_IRQHandler(&htim2);
}
