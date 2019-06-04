
#include "driver_i2c.h"
#include "global.h"

volatile u8 I2C_FastMode;

void I2C_Soft_delay()
{ 
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	
	if(!I2C_FastMode)
	{
		u8 i = 15;
		while(i--);
	}
}

void I2C_Soft_Init(u8 i2c_channel)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);
    switch(i2c_channel)
    {
    case I2C_1:
        GPIO_InitStructure.GPIO_Pin =  I2C1_Pin_SDA | I2C1_Pin_SCL;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(I2C1_GPIO, &GPIO_InitStructure);
        break;
    case I2C_2:
        GPIO_InitStructure.GPIO_Pin =  I2C2_Pin_SDA | I2C2_Pin_SCL;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(I2C2_GPIO, &GPIO_InitStructure);
        break;
    case I2C_3:
        GPIO_InitStructure.GPIO_Pin =  I2C3_Pin_SCL;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(I2C3_GPIO_SCL, &GPIO_InitStructure);
    
        GPIO_InitStructure.GPIO_Pin =  I2C3_Pin_SDA;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(I2C3_GPIO_SDA, &GPIO_InitStructure);
        break;
    default:
        break;        
    }
}


int I2C_Soft_Start(u8 i2c_channel)
{
    SCL_L(i2c_channel);
    SDA_H(i2c_channel);
    I2C_Soft_delay();
    SCL_H(i2c_channel);
    I2C_Soft_delay();
    if(!SDA_read(i2c_channel))return 0;	//SDA为低电平则总线忙 退出
    SDA_L(i2c_channel);	
    if(SDA_read(i2c_channel)) return 0;	//SDA为高电平则总线错误 退出
    SDA_L(i2c_channel);
    I2C_Soft_delay();
    SCL_L(i2c_channel);//钳住I2C总线，准备发送或接收数据 
    return 1;
}

void I2C_Soft_Stop(u8 i2c_channel)
{
    I2C_Soft_delay();
	SCL_L(i2c_channel);
	I2C_Soft_delay();
	SDA_L(i2c_channel);
	I2C_Soft_delay();
	SCL_H(i2c_channel);
	I2C_Soft_delay();
	SDA_H(i2c_channel);
	I2C_Soft_delay();
}

void I2C_Soft_Ask(u8 i2c_channel)
{
	SCL_L(i2c_channel);
	I2C_Soft_delay();
	SDA_L(i2c_channel);
	I2C_Soft_delay();
	SCL_H(i2c_channel);
	I2C_Soft_delay();
	SCL_L(i2c_channel);
	I2C_Soft_delay();
}

void I2C_Soft_NoAsk(u8 i2c_channel)
{
	SCL_L(i2c_channel);
	I2C_Soft_delay();
	SDA_H(i2c_channel);
	I2C_Soft_delay();
	SCL_H(i2c_channel);
	I2C_Soft_delay();
	SCL_L(i2c_channel);
	I2C_Soft_delay();
}

int I2C_Soft_WaitAsk(u8 i2c_channel) 	 //返回值为:=1 无ASK ,=0有ASK
{
    u8 ErrTime = 0;
	SCL_L(i2c_channel);
	I2C_Soft_delay();
	SDA_H(i2c_channel);			
	I2C_Soft_delay();
	SCL_H(i2c_channel);
	I2C_Soft_delay();
    I2C_Soft_delay();
    I2C_Soft_delay();
    I2C_Soft_delay();
    I2C_Soft_delay();
	while(SDA_read(i2c_channel))
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2C_Soft_Stop(i2c_channel);
			return 1;
		}
        I2C_Soft_delay();
	}
	SCL_L(i2c_channel);
	I2C_Soft_delay();
	return 0;
}

void I2C_Soft_SendByte(u8 i2c_channel, u8 SendByte) //数据从高地到低位
{
    u8 i=8;
    while(i--)
    {
        SCL_L(i2c_channel);
        I2C_Soft_delay();
      if(SendByte&0x80)
        SDA_H(i2c_channel);  
      else 
        SDA_L(i2c_channel);   
        SendByte<<=1;
        I2C_Soft_delay();
				SCL_H(i2c_channel);
				I2C_Soft_delay();
    }
    SCL_L(i2c_channel);
}  

//读1个字节时,ack=1时,发送ACK,ack=0,发送NACK
u8 I2C_Soft_ReadByte(u8 i2c_channel, u8 ask)  //数据从高位到低位//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H(i2c_channel);				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L(i2c_channel);
      I2C_Soft_delay();
			SCL_H(i2c_channel);
      I2C_Soft_delay();	
      if(SDA_read(i2c_channel))
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L(i2c_channel);

	if (ask)
		I2C_Soft_Ask(i2c_channel);
	else
		I2C_Soft_NoAsk(i2c_channel);  
    return ReceiveByte;
} 


// IIC写1字节数据
u8 IIC_Write_1Byte(u8 i2c_channel, u8 SlaveAddress, u8 REG_Address, u8 REG_data)
{
	I2C_Soft_Start(i2c_channel);
	I2C_Soft_SendByte(i2c_channel, SlaveAddress<<1);   
	if(I2C_Soft_WaitAsk(i2c_channel))
	{
		I2C_Soft_Stop(i2c_channel);
		return 1;
	}
	I2C_Soft_SendByte(i2c_channel, REG_Address);       
	I2C_Soft_WaitAsk(i2c_channel);	
	I2C_Soft_SendByte(i2c_channel, REG_data);
	I2C_Soft_WaitAsk(i2c_channel);   
	I2C_Soft_Stop(i2c_channel); 
	return 0;
}

// IIC读1字节数据
u8 IIC_Read_1Byte(u8 i2c_channel, u8 SlaveAddress, u8 REG_Address, u8 *REG_data)
{      		
	I2C_Soft_Start(i2c_channel);
	I2C_Soft_SendByte(i2c_channel, SlaveAddress<<1); 
	if(I2C_Soft_WaitAsk(i2c_channel))
	{
		I2C_Soft_Stop(i2c_channel);
		return 1;
	}
	I2C_Soft_SendByte(i2c_channel, REG_Address);     
	I2C_Soft_WaitAsk(i2c_channel);
	I2C_Soft_Start(i2c_channel);
	I2C_Soft_SendByte(i2c_channel, SlaveAddress<<1 | 0x01);
	I2C_Soft_WaitAsk(i2c_channel);
	*REG_data= I2C_Soft_ReadByte(i2c_channel, 0);
	I2C_Soft_Stop(i2c_channel);
	return 0;
}	

// IIC写n字节数据
u8 IIC_Write_nByte(u8 i2c_channel, u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	I2C_Soft_Start(i2c_channel);
	I2C_Soft_SendByte(i2c_channel, SlaveAddress<<1); 
	if(I2C_Soft_WaitAsk(i2c_channel))
	{
		I2C_Soft_Stop(i2c_channel);
		return 1;
	}
	I2C_Soft_SendByte(i2c_channel, REG_Address); 
	I2C_Soft_WaitAsk(i2c_channel);
	while(len--) 
	{
		I2C_Soft_SendByte(i2c_channel, *buf++); 
		I2C_Soft_WaitAsk(i2c_channel);
	}
	I2C_Soft_Stop(i2c_channel);
	return 0;
}

// IIC读n字节数据
u8 IIC_Read_nByte(u8 i2c_channel, u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	I2C_Soft_Start(i2c_channel);
	I2C_Soft_SendByte(i2c_channel, SlaveAddress<<1); 
	if(I2C_Soft_WaitAsk(i2c_channel))
	{
		I2C_Soft_Stop(i2c_channel);
		return 1;
	}
	I2C_Soft_SendByte(i2c_channel, REG_Address); 
	I2C_Soft_WaitAsk(i2c_channel);
	
	I2C_Soft_Start(i2c_channel);
	I2C_Soft_SendByte(i2c_channel, SlaveAddress<<1 | 0x01); 
	I2C_Soft_WaitAsk(i2c_channel);
	while(len) 
	{
		if(len == 1)
		{
			*buf = I2C_Soft_ReadByte(i2c_channel, 0);
		}
		else
		{
			*buf = I2C_Soft_ReadByte(i2c_channel, 1);
		}
		buf++;
		len--;
	}
	I2C_Soft_Stop(i2c_channel);
	return 0;
}
