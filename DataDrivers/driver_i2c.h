
#ifndef DRIVER_I2C_H
#define	DRIVER_I2C_H

#include "stm32f4xx.h"

/***************I2C GPIO¶¨Òå******************/
#define I2C1_GPIO	        GPIOB
#define I2C1_Pin_SCL		GPIO_Pin_6
#define I2C1_Pin_SDA		GPIO_Pin_7

#define I2C2_GPIO	        GPIOB
#define I2C2_Pin_SCL		GPIO_Pin_10
#define I2C2_Pin_SDA		GPIO_Pin_11

#define I2C3_GPIO_SCL	    GPIOA
#define I2C3_GPIO_SDA	    GPIOC
#define I2C3_Pin_SCL		GPIO_Pin_8
#define I2C3_Pin_SDA		GPIO_Pin_9
/*********************************************/

/***************I2C GPIO¸´ÓÃ******************/
#define SCL_H(x)    ( (x) == I2C_1? (I2C1_GPIO->BSRRL = I2C1_Pin_SCL): \
                                    ( (x) == I2C_2? (I2C2_GPIO->BSRRL = I2C2_Pin_SCL): \
                                                    (I2C3_GPIO_SCL->BSRRL = I2C3_Pin_SCL) ) )
#define SCL_L(x)    ( (x) == I2C_1? (I2C1_GPIO->BSRRH = I2C1_Pin_SCL): \
                                    ( (x) == I2C_2? (I2C2_GPIO->BSRRH = I2C2_Pin_SCL): \
                                                    (I2C3_GPIO_SCL->BSRRH = I2C3_Pin_SCL) ) )    
#define SDA_H(x)    ( (x) == I2C_1? (I2C1_GPIO->BSRRL = I2C1_Pin_SDA): \
                                    ( (x) == I2C_2? (I2C2_GPIO->BSRRL = I2C2_Pin_SDA): \
                                                    (I2C3_GPIO_SDA->BSRRL = I2C3_Pin_SDA) ) )       
#define SDA_L(x)    ( (x) == I2C_1? (I2C1_GPIO->BSRRH = I2C1_Pin_SDA): \
                                    ( (x) == I2C_2? (I2C2_GPIO->BSRRH = I2C2_Pin_SDA): \
                                                    (I2C3_GPIO_SDA->BSRRH = I2C3_Pin_SDA) ) )                                                     
#define SCL_read(x)    ( (x) == I2C_1? (I2C1_GPIO->IDR & I2C1_Pin_SCL): \
                                       ( (x) == I2C_2? (I2C2_GPIO->IDR & I2C2_Pin_SCL): \
                                                       (I2C3_GPIO_SCL->IDR & I2C3_Pin_SCL) ) ) 
#define SDA_read(x)    ( (x) == I2C_1? (I2C1_GPIO->IDR & I2C1_Pin_SDA): \
                                       ( (x) == I2C_2? (I2C2_GPIO->IDR & I2C2_Pin_SDA): \
                                                       (I2C3_GPIO_SDA->IDR & I2C3_Pin_SDA) ) )                                                        

//#define I2C1_SCL_H         I2C1_GPIO->BSRRL = I2C1_Pin_SCL
//#define I2C1_SCL_L         I2C1_GPIO->BSRRH = I2C1_Pin_SCL
//#define I2C1_SDA_H         I2C1_GPIO->BSRRL = I2C1_Pin_SDA
//#define I2C1_SDA_L         I2C1_GPIO->BSRRH = I2C1_Pin_SDA
//#define I2C1_SCL_read      I2C1_GPIO->IDR & I2C1_Pin_SCL
//#define I2C1_SDA_read      I2C1_GPIO->IDR & I2C1_Pin_SDA

//#define I2C2_SCL_H         I2C2_GPIO->BSRRL = I2C2_Pin_SCL
//#define I2C2_SCL_L         I2C2_GPIO->BSRRH = I2C2_Pin_SCL
//#define I2C2_SDA_H         I2C2_GPIO->BSRRL = I2C2_Pin_SDA
//#define I2C2_SDA_L         I2C2_GPIO->BSRRH = I2C2_Pin_SDA
//#define I2C2_SCL_read      I2C2_GPIO->IDR & I2C2_Pin_SCL
//#define I2C2_SDA_read      I2C2_GPIO->IDR & I2C2_Pin_SDA

//#define I2C3_SCL_H         I2C3_GPIO_SCL->BSRRL = I2C3_Pin_SCL
//#define I2C3_SCL_L         I2C3_GPIO_SCL->BSRRH = I2C3_Pin_SCL
//#define I2C3_SDA_H         I2C3_GPIO_SDA->BSRRL = I2C3_Pin_SDA
//#define I2C3_SDA_L         I2C3_GPIO_SDA->BSRRH = I2C3_Pin_SDA
//#define I2C3_SCL_read      I2C3_GPIO_SCL->IDR & I2C3_Pin_SCL
//#define I2C3_SDA_read      I2C3_GPIO_SDA->IDR & I2C3_Pin_SDA
/*********************************************/

#ifdef  __cplusplus  
extern "C" {  
#endif     
void I2C_Soft_Init(u8 i2c_channel);
void I2C_Soft_SendByte(u8 i2c_channel,u8 SendByte);
u8 I2C_Soft_ReadByte(u8 i2c_channel, u8 ask);
    
u8 IIC_Write_1Byte(u8 i2c_channel, u8 SlaveAddress, u8 REG_Address, u8 REG_data);
u8 IIC_Read_1Byte(u8 i2c_channel, u8 SlaveAddress, u8 REG_Address, u8 *REG_data);
u8 IIC_Write_nByte(u8 i2c_channel, u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nByte(u8 i2c_channel, u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);    
#ifdef  __cplusplus  
}  
#endif  
extern volatile u8 I2C_FastMode;

#endif //DRIVER_I2C_H
