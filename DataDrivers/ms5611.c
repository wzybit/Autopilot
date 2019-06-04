
#include "ms5611.h"
#include "driver_i2c.h"
#include "utility.h"
#include "global.h"

int32_t height;
uint16_t ms5611_prom[PROM_NB];  // on-chip ROM
uint8_t t_rxbuf[3],p_rxbuf[3];

void MS5611_Reset(void)
{
    IIC_Write_1Byte(I2C_1, MS5611_ADDR, CMD_RESET, 1);
}

u8 MS5611_Read_Prom(void)
{
	uint8_t rxbuf[2] = { 0, 0 };
	u8 check = 0;
	u8 i;

	for (i = 0; i < PROM_NB; i++)
	{
		check += IIC_Read_nByte(I2C_1, MS5611_ADDR, CMD_PROM_RD + i * 2, 2, rxbuf); // send PROM READ command
		ms5611_prom[i] = rxbuf[0] << 8 | rxbuf[1];
	}

	if(check==PROM_NB)
		return 1;
	else
		return 0;
}

void MS5611_Read_Adc_T(void)
{
	IIC_Read_nByte(I2C_1, MS5611_ADDR, CMD_ADC_READ, 3, t_rxbuf); // read ADC
}

void MS5611_Read_Adc_P(void)
{
	IIC_Read_nByte(I2C_1, MS5611_ADDR, CMD_ADC_READ, 3, p_rxbuf); // read ADC
}

void MS5611_Start_T(void)
{
	IIC_Write_1Byte(I2C_1, MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 1); // D2 (temperature) conversion start!
}

void MS5611_Start_P(void)
{
    IIC_Write_1Byte(I2C_1, MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 1); // D1 (pressure) conversion start!
}

void MS5611_Init(void)
{	
	delay_ms(10);	
	MS5611_Reset();//传感器复位
	delay_ms(3);
	globalFlag.baro_ok = !(MS5611_Read_Prom());	
	MS5611_Start_T();//开始读取温度
}

void MS5611_BaroAltCalculate(void)
{
    static uint32_t ms5611_ut;  // static result of temperature measurement
    static uint32_t ms5611_up;  // static result of pressure measurement
    	
    int32_t temperature, off2 = 0, sens2 = 0, delt;
    float pressure;
	float alt_3;
	
	int32_t dT;
	int64_t off;
	int64_t sens;
		
    ms5611_ut = (t_rxbuf[0] << 16) | (t_rxbuf[1] << 8) | t_rxbuf[2];
    ms5611_up = (p_rxbuf[0] << 16) | (p_rxbuf[1] << 8) | p_rxbuf[2];
		
    dT = ms5611_ut - ((uint32_t)ms5611_prom[5] << 8);
    off = ((uint32_t)ms5611_prom[2] << 16) + (((int64_t)dT * ms5611_prom[4]) >> 7);
    sens = ((uint32_t)ms5611_prom[1] << 15) + (((int64_t)dT * ms5611_prom[3]) >> 8);
    temperature = 2000 + (((int64_t)dT * ms5611_prom[6]) >> 23);

    if (temperature < 2000) 
    { // temperature lower than 20degC 
        delt = temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        if (temperature < -1500) 
        { // temperature lower than -15degC
            delt = temperature + 1500;
            delt = delt * delt;
            off2  += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
    }
    off  -= off2; 
    sens -= sens2;
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
		
    alt_3 = (101000.0f - pressure)/1000.0f;
    pressure = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * (101000 - pressure)*100.0f ;
		
    height = (int32_t)pressure; //cm +        
}

int32_t MS5611_Read(void)
{
	static int state = 0;
	
	I2C_FastMode = 0;
	
	if (state) 
	{
			MS5611_Read_Adc_P();
			MS5611_Start_T();
			MS5611_BaroAltCalculate();
			state = 0;
	} 
	else 
	{
			MS5611_Read_Adc_T();
			MS5611_Start_P();
			state = 1;
	}
    return height;
}
