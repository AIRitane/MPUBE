#include "bsp_iic.h"                  // Device header
#include "gpio.h"

/*
for循环实现延时us
*/
void Delay_us(uint32_t nus)
{
    uint32_t Delay = nus;
    do
    {
        __NOP();
    }
    while (Delay --);
}


/*
封装三个函数操作相应的IO口便于移植
MyI2C_W_SCL
MyI2C_W_SDA
MyI2C_R_SDA
*/
void MyI2C_W_SCL(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,BitValue);
	Delay_us(20);
}

void MyI2C_W_SDA(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,BitValue);
	Delay_us(20);
}

uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
	Delay_us(20);
	return BitValue;
}

void MyI2C_Init(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
}

void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);//为了兼容sr重复起始条件，先释放sda后释放scl
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
}

void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);//为了确保释放sda是高电平，先拉低sda
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SDA(Byte & (0x80 >> i));//高位先行，按位与取出相应位数，右移得到相应的位
		MyI2C_W_SCL(1);//高位读取sda数据
		MyI2C_W_SCL(0);
	}
}

uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;//定义一个字节的数据
	MyI2C_W_SDA(1);//主机释放sda
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SCL(1);//scl高电平时放入数据
		if (MyI2C_R_SDA() == 1){
			Byte |= (0x80 >> i);//输出寄存器为1则在该字节的相应位置写入1
		}
		MyI2C_W_SCL(0);
	}
	return Byte;
}
//发送用应答
void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}
//接收应答
uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;
	MyI2C_W_SDA(1);//释放sda，而不是输出1
	MyI2C_W_SCL(1);
	AckBit = MyI2C_R_SDA();//读到0则表示从机应答
	MyI2C_W_SCL(0);
	return AckBit;
}


