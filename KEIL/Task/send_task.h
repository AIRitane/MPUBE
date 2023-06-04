#ifndef _SEND_TASK_H
#define _SEND_TASK_H

#include "stm32f4xx_hal.h"
#include "motion_task.h"

#define MAX_BUFFER 128


#define STATE 0x00
#define PITCH 0x01
#define ROLL 0x02
#define TOP 0x03
#define TEST2 0x04
#define TEST3 0x05
#define MODE 0x06

typedef struct
{
	int int_data;
	int dec_data;
	int pol_data;
}float_data;


void HC08_SendData(uint8_t* dataBuf, uint16_t cnt);
void HC08_SendString(uint8_t *strTemp, uint32_t len);
void Data_package(uint8_t type, uint8_t data);
float_data float_data_handle(float cube_data, float_data data);

#endif