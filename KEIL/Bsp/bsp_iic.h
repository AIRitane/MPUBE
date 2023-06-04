#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "stm32f4xx_hal.h"
#define IIC_SCL_GPIO_Port GPIOA
#define IIC_SDA_GPIO_Port GPIOC
#define IIC_SCL_Pin_Num 8
#define IIC_SDA_Pin_Num 12

#define IIC_Pin(num) (1<<(num))
#define IIC_SCL_Pin IIC_Pin(IIC_SCL_Pin_Num)
#define IIC_SDA_Pin IIC_Pin(IIC_SDA_Pin_Num)

#define SDA_IN()  {IIC_SDA_GPIO_Port->MODER&=~(3<<(IIC_SDA_Pin_Num*2));IIC_SDA_GPIO_Port->MODER|=0<<(IIC_SDA_Pin_Num*2);}	
#define SDA_OUT() {IIC_SDA_GPIO_Port->MODER&=~(3<<(IIC_SDA_Pin_Num*2));IIC_SDA_GPIO_Port->MODER|=1<<(IIC_SDA_Pin_Num*2);}

#define IIC_SCL(state) (HAL_GPIO_WritePin(IIC_SCL_GPIO_Port,IIC_SCL_Pin,(state)))
#define IIC_SDA(state) (HAL_GPIO_WritePin(IIC_SDA_GPIO_Port,IIC_SDA_Pin,(state)))
#define READ_SDA() (HAL_GPIO_ReadPin(IIC_SDA_GPIO_Port,IIC_SDA_Pin))

void IIC_Init(void);               				 
void IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(uint8_t txd);		
uint8_t IIC_Read_Byte(unsigned char ack);
uint8_t IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);				
	 
#endif


