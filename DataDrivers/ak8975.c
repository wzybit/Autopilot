
#include "ak8975.h"
#include "driver_i2c.h"
#include "global.h"

s16 mag_val[3];
u8 ak8975_buffer[6];

u8 AK8975_Init(void)
{
	 return (!(IIC_Write_1Byte(I2C_2, AK8975_ADDRESS, AK8975_CNTL, 0x01)));	
}

void AK8975_Read(void)
{			
	I2C_FastMode = 0;
	
	IIC_Read_1Byte(I2C_2, AK8975_ADDRESS, AK8975_HXL, &ak8975_buffer[0]); 
	IIC_Read_1Byte(I2C_2, AK8975_ADDRESS, AK8975_HXH, &ak8975_buffer[1]);  //磁力计X轴

	IIC_Read_1Byte(I2C_2, AK8975_ADDRESS, AK8975_HYL, &ak8975_buffer[2]);
	IIC_Read_1Byte(I2C_2, AK8975_ADDRESS, AK8975_HYH, &ak8975_buffer[3]); //磁力计Y轴

	IIC_Read_1Byte(I2C_2, AK8975_ADDRESS, AK8975_HZL, &ak8975_buffer[4]);
	IIC_Read_1Byte(I2C_2, AK8975_ADDRESS, AK8975_HZH, &ak8975_buffer[5]);  //磁力计Z轴	
	
	//AK8975采样触发
	IIC_Write_1Byte(I2C_2, AK8975_ADDRESS, AK8975_CNTL, 0x01);
}

void Mag_Get(void)
{
	s16 t[3];
	
	t[0] = ((((int16_t)ak8975_buffer[1]) << 8) | ak8975_buffer[0]) ;
	t[1] = ((((int16_t)ak8975_buffer[3]) << 8) | ak8975_buffer[2]) ;
	t[2] = ((((int16_t)ak8975_buffer[5]) << 8) | ak8975_buffer[4]) ;
	
	/*转换坐标轴为ANO坐标*/
	mag_val[0] = +t[0];
	mag_val[1] = -t[1];
	mag_val[2] = -t[2];
}
