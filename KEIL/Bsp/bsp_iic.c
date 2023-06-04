#include "bsp_iic.h"                  // Device header
#include "gpio.h"

/*
for循环实现延时us
*/
void delay_us(uint32_t nus)
{
    uint32_t Delay = nus;
    do
    {
        __NOP();
    }
    while (Delay --);
}

// change it
void IIC_Init(void)
{
	IIC_SCL(1);
	IIC_SDA(1);
}


void IIC_Start(void)
{
	SDA_OUT();
	IIC_SDA(1);
	IIC_SCL(1);
	delay_us(4);
	IIC_SDA(0); // START:when CLK is high,DATA change form high to low
	delay_us(4);
	IIC_SCL(0);
}

void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL(0);
	IIC_SDA(0);
	delay_us(4);
	IIC_SCL(1);
	IIC_SDA(1);
	delay_us(4);
}

uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime = 0;
	SDA_IN();
	IIC_SDA(1);
	delay_us(1);
	IIC_SCL(1);
	delay_us(1);
	while (READ_SDA())
	{
		ucErrTime++;
		if (ucErrTime > 250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL(0);
	return 0;
}

void IIC_Ack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(0);
	delay_us(2);
	IIC_SCL(1);
	delay_us(2);
	IIC_SCL(0);
}

void IIC_NAck(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(1);
	delay_us(2);
	IIC_SCL(1);
	delay_us(2);
	IIC_SCL(0);
}

void IIC_Send_Byte(uint8_t txd)
{
	uint8_t t;
	SDA_OUT();
	IIC_SCL(0);
	for (t = 0; t < 8; t++)
	{
		IIC_SDA((txd & 0x80) >> 7);
		txd <<= 1;
		delay_us(2);
		IIC_SCL(1);
		delay_us(2);
		IIC_SCL(0);
		delay_us(2);
	}
}

uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA_IN();
	for (i = 0; i < 8; i++)
	{
		IIC_SCL(0);
		delay_us(2);
		IIC_SCL(1);
		receive <<= 1;
		if (READ_SDA())
			receive++;
		delay_us(1);
	}
	if (!ack)
		IIC_NAck();
	else
		IIC_Ack();
	return receive;
}
