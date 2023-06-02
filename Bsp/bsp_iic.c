#include "bsp_iic.h"                  // Device header
#include "gpio.h"

/*
forѭ��ʵ����ʱus
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
��װ��������������Ӧ��IO�ڱ�����ֲ
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
	MyI2C_W_SDA(1);//Ϊ�˼���sr�ظ���ʼ���������ͷ�sda���ͷ�scl
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
}

void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);//Ϊ��ȷ���ͷ�sda�Ǹߵ�ƽ��������sda
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SDA(Byte & (0x80 >> i));//��λ���У���λ��ȡ����Ӧλ�������Ƶõ���Ӧ��λ
		MyI2C_W_SCL(1);//��λ��ȡsda����
		MyI2C_W_SCL(0);
	}
}

uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;//����һ���ֽڵ�����
	MyI2C_W_SDA(1);//�����ͷ�sda
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SCL(1);//scl�ߵ�ƽʱ��������
		if (MyI2C_R_SDA() == 1){
			Byte |= (0x80 >> i);//����Ĵ���Ϊ1���ڸ��ֽڵ���Ӧλ��д��1
		}
		MyI2C_W_SCL(0);
	}
	return Byte;
}
//������Ӧ��
void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}
//����Ӧ��
uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;
	MyI2C_W_SDA(1);//�ͷ�sda�����������1
	MyI2C_W_SCL(1);
	AckBit = MyI2C_R_SDA();//����0���ʾ�ӻ�Ӧ��
	MyI2C_W_SCL(0);
	return AckBit;
}


