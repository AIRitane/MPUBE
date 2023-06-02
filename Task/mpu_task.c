#include "cmsis_os.h"
#include "mpu_task.h"
#include "bsp_mpu6050.h"

int16_t AX, AY, AZ, GX, GY, GZ;

void mpu_task(void const * argument)
{
	osDelay(2000); // 该延迟很重要，mpu6050需要上电延时准备
	MPU6050_Init();
	while(MPU6050_GetID() != 0x68)
	{
		osDelay(1000);
	}
	
	while(1)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		
		osDelay(100);
	}
}
