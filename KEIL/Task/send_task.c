#include "cmsis_os.h"
#include "send_task.h"


cube_t cube_now;
cube_t cube_last;
float_data float_pitch;
float_data float_roll;

uint8_t buffer[MAX_BUFFER];

void send_task()
{
	while(1)
	{
		cube_last = cube_now;
		cube_now = cube;
		float_pitch = float_data_handle(cube_now.pitch, float_pitch);
		float_roll = float_data_handle(cube_now.roll, float_roll);

		osDelay(100);
		
		if((cube_last.pitch - cube_now.pitch)<1.0 && (cube_last.pitch - cube_now.pitch)>-1.0)
		{
			buffer[0] = 0x55;
			buffer[1] = PITCH;
			buffer[2] = float_pitch.pol_data;
			buffer[3] = float_pitch.int_data;
			buffer[4] = float_pitch.dec_data;
			buffer[5] = 0x56;
			HC08_SendString(buffer, 6);
			osDelay(100);
		}
		
		if((cube_last.roll - cube_now.roll)<1.0 && (cube_last.roll - cube_now.roll)>-1.0)
		{
			buffer[0] = 0x55;
			buffer[1] = ROLL;
			buffer[2] = float_roll.pol_data;
			buffer[3] = float_roll.int_data;
			buffer[4] = float_roll.dec_data;
			buffer[5] = 0x56;
			HC08_SendString(buffer, 6);
			osDelay(100);
		}
		
		Data_package(TOP, cube.top);
		HC08_SendString(buffer, 4);
		osDelay(100);
		
		Data_package(MODE, cube.mode);
		HC08_SendString(buffer, 4);
		osDelay(100);
	}
}

void HC08_SendString(uint8_t *strTemp, uint32_t len)          
{
    HC08_SendData(strTemp, len);                  
}

void HC08_SendData(uint8_t* dataBuf, uint16_t cnt)  
{
    while(cnt--)
    {
        while((USART3->SR & 0X40)==0);
        USART3->DR = *dataBuf;
        dataBuf++;
    }
}

void Data_package(uint8_t type, uint8_t data)
{
		buffer[0] = 0x55;
		buffer[1] = type;
		buffer[2] = data;
		buffer[3] = 0x56;
}

float_data float_data_handle(float cube_data, float_data data)
{
		if(cube_data > 0) data.pol_data = 1;
		else if(cube_data <= 0)
		{
			cube_data = -cube_data;
			data.pol_data = 0;
		}
	
		data.int_data = cube_data;
		data.dec_data = (cube_data - data.int_data) * 100;
		
	
		return data;
}