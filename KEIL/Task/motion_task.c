#include "cmsis_os.h"
#include "motion_task.h"
#include "mpu_task.h"
#include "math.h"
#include "usart.h"
#include "kalman.h"

// 滤波器BUFFER
float task3_acc_filter_buffer[2][TASK3_ACC_BUFFER_LEN] = {0};
smooth_filter_t task3_acc_filter[2] = {0};

IMU_Info *imu_now;
cube_t cube = {0};
uint8_t mode_buffer[10];

// 地图
uint8_t map[4][4] = {{0x00, 0x01, 0x02, 0x03},
					 {0x04, 0x05, 0x06, 0x07},
					 {0x08, 0x09, 0x10, 0x11},
					 {0x12, 0x13, 0x14, 0x15}};

// 盒子模型
uint8_t cube_dic[6] = {1, 2, 3, 4, 5, 6};			// 前后左右上下
uint8_t DEBUG_cube_dic = 0;							// 移动数据出错检测标志位
int point[4][2] = {{0, 0}, {0, 3}, {3, 0}, {3, 3}}; // 四角点
uint8_t hot_code_point[4] = {0};					// 四角点倒退错误

float FMABS(float x)
{
	x = x > 0 ? x : -x;
	return x;
}

// 已验证，旋转盒子模型
void refresh_cube_dic(uint8_t cube_dic[], uint8_t mode)
{
	uint8_t temp = 0;
	if (mode == 0) // 前
	{
		temp = cube_dic[0];
		cube_dic[0] = cube_dic[4];
		cube_dic[4] = cube_dic[1];
		cube_dic[1] = cube_dic[5];
		cube_dic[5] = temp;
	}

	else if (mode == 1) // 后
	{
		temp = cube_dic[0];
		cube_dic[0] = cube_dic[5];
		cube_dic[5] = cube_dic[1];
		cube_dic[1] = cube_dic[4];
		cube_dic[4] = temp;
	}

	else if (mode == 2) // 左
	{
		temp = cube_dic[3];
		cube_dic[3] = cube_dic[5];
		cube_dic[5] = cube_dic[2];
		cube_dic[2] = cube_dic[4];
		cube_dic[4] = temp;
	}

	else if (mode == 3) // 右
	{
		temp = cube_dic[3];
		cube_dic[3] = cube_dic[4];
		cube_dic[4] = cube_dic[2];
		cube_dic[2] = cube_dic[5];
		cube_dic[5] = temp;
	}
}

// 解算盒子状态
void get_cube_status(uint8_t cube_now_status)
{
	static uint8_t map_l_index = 0,map_r_index = 0;
	uint8_t tmp_cube_dic = 0;
	for (uint8_t i = 0; i < 4; i++)
	{
		// 查找
		if (cube_dic[i] == cube_now_status)
		{
			// 数组坐标
			if (i == 0)
			{
				refresh_cube_dic(cube_dic, 1); // 原盒子前面向上，需要原盒子后旋
				map_l_index--;
			}
			else if (i == 1)
			{
				refresh_cube_dic(cube_dic, 0);
				map_l_index++;
			}
			else if (i == 2)
			{
				refresh_cube_dic(cube_dic, 3);
				map_r_index++;
			}
			else if (i == 3)
			{
				refresh_cube_dic(cube_dic, 2);
				map_r_index--;
			}

			// 发送坐标
			Data_package_mode(TEST2, map[map_l_index][map_r_index]);
			HC08_SendString_mode(mode_buffer, 4);

			tmp_cube_dic = 1;
			break;
		}
	}
	if (!tmp_cube_dic)
	{
		// 用于调试
		DEBUG_cube_dic = 1;
	}
}

void motion_task(void const *argument)
{
	// shake 状态标志位
	uint32_t shake_count = 0;
	uint8_t shake_flag = 0;

	// task1
	uint8_t task1_change_flag = 0;
	uint32_t task1_change_count = 0;

	// task2
	uint8_t task2_change_flag = 0;
	uint32_t task2_change_count = 0;
	uint8_t task2_is_frist_in = 0;

	// task3
	uint8_t task3_change_flag = 0;
	uint8_t task3_is_frist_in = 0;
	uint8_t task3_fist_in = 0;
	uint32_t task3_change_count = 0;
	float task3_average_acc[3] = {0.0f};

	uint8_t task3_acc_raw_is_refresh = 0;

	uint8_t task3_wave[2][2] = {0}; // 上波腹 、 下波腹
	uint32_t task3_wave_count[2][2] = {0};
	uint8_t task3_wave_raw_refresh[2][2] = {0};

	// 霍尔传感器管脚初始化
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOE_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	// 初始开关状态发送
	Data_package_mode(STATE, 0x00);
	HC08_SendString_mode(mode_buffer, 4);

	// 滤波器初始化
	for (uint8_t i = 0; i < 2; i++)
	{
		task3_acc_filter[i].buffer = task3_acc_filter_buffer[i];
		task3_acc_filter[i].buffer_len = TASK3_ACC_BUFFER_LEN;
	}

	// 初始化变量
	imu_now = IMU_GetInfo();
	cube.mode = no_mode;
	IMU_Set_Is_Shake(0);

	// 延迟等待马霍妮积分收敛
	osDelay(3000);

	while (1)
	{
		// 无模式，该模式0x10 表示准备 0x00表示进入无模式
		// 有四秒时间准备
		if (cube.mode == no_mode)
		{
			// 检测晃动
			if (imu_now->is_shake && !shake_flag)
			{
				IMU_Set_Is_Shake(0);
				shake_flag = 1;
			}

			if (shake_flag)
			{
				shake_count++;
				if (imu_now->is_shake)
				{
					IMU_Set_Is_Shake(0);
					shake_count = 0;
				}

				if (shake_count >= 1000)
				{
					// 发送标识
					Data_package_mode(MODE, 0x10);
					HC08_SendString_mode(mode_buffer, 4);

					// 发送ON状态
					Data_package_mode(STATE, 0x01);
					HC08_SendString_mode(mode_buffer, 4);

					osDelay(3000);

					// 发送标识
					Data_package_mode(MODE, 0x01);
					HC08_SendString_mode(mode_buffer, 4);
					set_mpu_base();

					shake_flag = 0;
					shake_count = 0;
					cube.mode = mode1;
				}
			}
		}

		// 进入无模式
		else if (cube.mode == mode1)
		{
			if (imu_now->is_shake && !shake_flag)
			{
				IMU_Set_Is_Shake(0);
				shake_flag = 1;
			}

			if (shake_flag)
			{
				shake_count++;
				if (imu_now->is_shake)
				{
					IMU_Set_Is_Shake(0);
					shake_count = 0;
				}

				if (shake_count >= 1000)
				{
					// 发送标识
					Data_package_mode(MODE, 0x00);
					HC08_SendString_mode(mode_buffer, 4);

					// 发送OFF状态
					Data_package_mode(STATE, 0x00);
					HC08_SendString_mode(mode_buffer, 4);
					shake_flag = 0;
					shake_count = 0;
					cube.mode = no_mode;
				}
			}
		}

		// 模式一
		// 检测面和返回角度
		if (cube.mode == mode1)
		{
			// 切换状态
			if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET && !task1_change_flag)
			{
				task1_change_flag = 1;
			}
			if (task1_change_flag)
			{
				task1_change_count++;
				if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
				{
					task1_change_count = 0;
				}
				if (task1_change_count >= 1000)
				{
					// 发送标识
					Data_package_mode(MODE, 0x20);
					HC08_SendString_mode(mode_buffer, 4);
					osDelay(500);
					set_mpu_base();
					// 发送标识
					Data_package_mode(MODE, 0x02);
					HC08_SendString_mode(mode_buffer, 4);

					task1_change_flag = 0;
					task1_change_count = 0;
					cube.mode = mode2;
				}
			}

			// 逻辑函数
			cube.pitch = imu_now->Pitch;
			cube.roll = imu_now->Roll;
			if (imu_now->Pitch <= 15.0 && imu_now->Pitch >= -15.0 && imu_now->Roll <= 30.0 && imu_now->Roll >= -30.0)
			{
				cube.top = A;
			}
			else if (imu_now->Pitch <= 15.0 && imu_now->Pitch >= -15.0 && imu_now->Roll <= -70.0 && imu_now->Roll >= -110.0)
			{
				cube.top = B;
			}
			else if (imu_now->Pitch <= 15.0 && imu_now->Pitch >= -15.0 && imu_now->Roll >= 150.0 || imu_now->Roll <= -150.0)
			{
				cube.top = C;
			}
			else if (imu_now->Pitch <= 15.0 && imu_now->Pitch >= -15.0 && imu_now->Roll <= 110.0 && imu_now->Roll >= 70.0)
			{
				cube.top = D;
			}
			else if (imu_now->Pitch <= -70.0 && imu_now->Pitch >= -110.0)
			{
				cube.top = E;
			}
			else if (imu_now->Pitch <= 110.0 && imu_now->Pitch >= 70.0)
			{
				cube.top = F;
			}
		}

		// 模式二
		// 返回翻滚字符
		else if (cube.mode == mode2)
		{
			// 切换状态
			if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET && !task2_change_flag)
			{
				task2_change_flag = 1;
			}
			if (task2_change_flag)
			{
				task2_change_count++;
				if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
				{
					task2_change_count = 0;
				}
				if (task2_change_count >= 1000)
				{
					Data_package_mode(MODE, 0x30);
					HC08_SendString_mode(mode_buffer, 4);
					osDelay(500);
					Data_package_mode(MODE, 0x03);
					HC08_SendString_mode(mode_buffer, 4);
					set_mpu_base();

					task2_change_flag = 0;
					task2_change_count = 0;
					task2_is_frist_in = 0;
					cube.mode = mode3;
				}
			}

			// 逻辑函数
			if (task2_is_frist_in != 0 && imu_now->Pitch <= 15.0 && imu_now->Pitch >= -15.0 && imu_now->Roll <= 30.0 && imu_now->Roll >= -30.0)
			{
				task2_is_frist_in = 0;
				get_cube_status(5);
			}
			else if (task2_is_frist_in != 1 && imu_now->Pitch <= 15.0 && imu_now->Pitch >= -15.0 && imu_now->Roll <= -70.0 && imu_now->Roll >= -110.0)
			{
				task2_is_frist_in = 1;
				get_cube_status(3);
			}
			else if (task2_is_frist_in != 2 && imu_now->Pitch <= 15.0 && imu_now->Pitch >= -15.0 && imu_now->Roll >= 150.0 || imu_now->Roll <= -150.0)
			{
				task2_is_frist_in = 2;
				get_cube_status(6);
			}
			else if (task2_is_frist_in != 3 && imu_now->Pitch <= 15.0 && imu_now->Pitch >= -15.0 && imu_now->Roll <= 110.0 && imu_now->Roll >= 75.0)
			{
				task2_is_frist_in = 3;
				get_cube_status(4);
			}
			else if (task2_is_frist_in != 4 && imu_now->Pitch <= -70.0 && imu_now->Pitch >= -90.0)
			{
				task2_is_frist_in = 4;
				get_cube_status(1);
			}
			else if (task2_is_frist_in != 5 && imu_now->Pitch <= 90.0 && imu_now->Pitch >= 70.0)
			{
				task2_is_frist_in = 5;
				get_cube_status(2);
			}
		}

		// 模式三
		// 返回原始数据点
		else if (cube.mode == mode3)
		{
			// 无需切换状态
			// 逻辑函数
			if (imu_now->data_refresh)
			{
				IMU_Set_Is_Refresh(0);
				cube.ax = averageFilter(&task3_acc_filter[0], imu_now->ax);
				cube.ay = averageFilter(&task3_acc_filter[1], imu_now->ay);
				task3_acc_raw_is_refresh = 1;
			}

			if (task3_acc_raw_is_refresh == 1)
			{
				task3_acc_raw_is_refresh = 0;

				// 除偏差
				// 丢弃初始数值
				if (task3_change_count < TASK3_AVER_ACC_DROPOUT && task3_is_frist_in == 0)
				{
					task3_change_count++;
					continue;
				}
				// 求和
				else if (task3_change_count < (TASK3_AVER_ACC_DROPOUT + TASK3_AVER_ACC_NUM) && task3_is_frist_in == 0)
				{
					task3_change_count++;
					task3_average_acc[0] += cube.ax;
					task3_average_acc[1] += cube.ay;
					continue;
				}
				// 平均
				else if (task3_change_count == (TASK3_AVER_ACC_DROPOUT + TASK3_AVER_ACC_NUM) && task3_is_frist_in == 0)
				{
					task3_average_acc[0] /= 100;
					task3_average_acc[1] /= 100;
					task3_is_frist_in = 1;
					task3_change_count = 0;
					continue;
				}
				else if (task3_is_frist_in == 0)
				{
					continue;
				}

				// 更新修正数据
				cube.ax -= task3_average_acc[0];
				cube.ay -= task3_average_acc[1];
				cube.acc_is_refresh = 1;
			}
			else
				continue;

			//	一次数据产生的条件
			// 检测连续N个点高于阈值
			// 检测连续N个点低于阈值

			if (cube.acc_is_refresh == 1)
			{
				cube.acc_is_refresh = 0;

				// 筛选节点
				// ay上界
				if (cube.ay > TASK3_ACCY_MAX)
				{
					task3_wave_count[1][0]++;

					if (task3_wave_count[1][0] > TASK3_ACCY_POINT_MAX && task3_wave_raw_refresh[1][0] == 0)
					{
						task3_wave_raw_refresh[1][0] = 1;
						if (task3_wave[1][1] == 0)
						{
							task3_wave[1][0] = 1;
						}
						else
						{
							task3_wave[1][0] = 2;
						}
					}
				}
				else
				{
					task3_wave_raw_refresh[1][0] = 0;
					task3_wave_count[1][0] = 0;
				}

				// ay下界
				if (cube.ay < TASK3_ACCY_MIN)
				{
					task3_wave_count[1][1]++;

					if (task3_wave_count[1][1] < TASK3_ACCY_POINT_MIN && task3_wave_raw_refresh[1][1] == 0)
					{
						task3_wave_raw_refresh[1][1] = 1;
						if (task3_wave[1][0] == 0)
						{
							task3_wave[1][1] = 1;
						}
						else
						{
							task3_wave[1][1] = 2;
						}
					}
				}
				else
				{
					task3_wave_raw_refresh[1][1] = 0;
					task3_wave_count[1][1] = 0;
				}

				// ax上界
				if (cube.ax > TASK3_ACCX_MAX)
				{
					task3_wave_count[0][0]++;

					if (task3_wave_count[0][0] > TASK3_ACCX_POINT_MAX && task3_wave_raw_refresh[0][0] == 0)
					{
						task3_wave_raw_refresh[0][0] = 1;
						if (task3_wave[0][1] == 0)
						{
							task3_wave[0][0] = 1;
						}
						else
						{
							task3_wave[0][0] = 2;
						}
					}
				}
				else
				{
					task3_wave_raw_refresh[0][0] = 0;
					task3_wave_count[0][0] = 0;
				}

				// ax下界
				if (cube.ax > TASK3_ACCX_MIN)
				{
					task3_wave_count[0][1]++;

					if (task3_wave_count[0][1] < TASK3_ACCX_POINT_MIN && task3_wave_raw_refresh[0][1] == 0)
					{
						task3_wave_raw_refresh[0][1] = 1;
						if (task3_wave[0][0] == 0)
						{
							task3_wave[0][1] = 1;
						}
						else
						{
							task3_wave[0][1] = 2;
						}
					}
				}
				else
				{
					task3_wave_raw_refresh[0][1] = 0;
					task3_wave_count[0][1] = 0;
				}

				// 解算节点
				if (task3_wave[0][0] == 1 && task3_wave[0][1] == 2)
				{
					cube.move_y[cube.move_len]--;
					cube.move_len++;

					task3_wave[0][0] = 0;
					task3_wave[0][1] = 0;
					task3_fist_in = 0;
				}
				else if (task3_wave[0][0] == 2 && task3_wave[0][1] == 1)
				{
					cube.move_y[cube.move_len]++;
					cube.move_len++;

					task3_wave[0][0] = 0;
					task3_wave[0][1] = 0;
					task3_fist_in = 0;
				}

				if (task3_wave[1][0] == 1 && task3_wave[1][1] == 2)
				{
					cube.move_x[cube.move_len]++;
					cube.move_len++;

					task3_wave[1][0] = 0;
					task3_wave[1][1] = 0;
					task3_fist_in = 0;
				}
				else if (task3_wave[1][0] == 2 && task3_wave[1][1] == 1)
				{
					cube.move_x[cube.move_len]--;
					cube.move_len++;

					task3_wave[1][0] = 0;
					task3_wave[1][1] = 0;
					task3_fist_in = 0;
				}
			}

			// 解算坐标
			if (FMABS(cube.ax) < 250 && FMABS(cube.ay) < 250)
			{
				if (!task3_fist_in)
				{
					task3_change_count++;

					// 完成标致
					if (task3_change_count > 3000)
					{
						// 这里需要注意，结构体不初始化为0，默认初始化无符号型值不为0 ！！！！
						for (int8_t i = cube.move_len - 1; i >= 0; i--)
						{
							for (int8_t j = 0; j < 4; j++)
							{
								point[j][0] -= cube.move_x[i];
								point[j][1] -= cube.move_y[i];

								if (point[j][0] == -1 || point[j][0] == 4 || point[j][1] == -1 || point[j][1] == 4)
								{
									hot_code_point[j] = 1;
								}
							}
						}

						for (uint8_t j = 0; j < 4; j++)
						{
							if (hot_code_point[j] == 0)
							{
								Data_package_mode(TEST3, map[point[j][0]][point[j][1]]);
								HC08_SendString_mode(mode_buffer, 4);
								// 发送 map(point[j][0],point[j][1])
								break;
							}
							if (j == 3)
							{
								Data_package_mode(TEST3, 0x10);
								HC08_SendString_mode(mode_buffer, 4);
							}
						}

						// 完成
						cube.mode = stop_mode;
						task3_change_count = 0;
					}
				}
				else
				{
					task3_change_count = 0;
				}
			}
		}
		osDelay(1);
	}
}

void HC08_SendString_mode(uint8_t *strTemp, uint32_t len)
{
	HAL_UART_Transmit(&huart3, strTemp, len, 0xff);
}

void Data_package_mode(uint8_t type, uint8_t data)
{
	mode_buffer[0] = 0x55;
	mode_buffer[1] = type;
	mode_buffer[2] = data;
	mode_buffer[3] = 0x56;
}
