
#include "mpu6050.h"
#include "driver_i2c.h"
#include "utility.h"
#include "global.h"

u8 imu_mpu6050_buffer[14];

void MPU6050_Read(void)
{
    I2C_FastMode = 1;
    IIC_Read_nByte(I2C_2, MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H, 14, imu_mpu6050_buffer);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	  �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1
ʧ��Ϊ0
*******************************************************************************/
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data) {
    u8 b;
    IIC_Read_nByte(I2C_2, dev, reg, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    globalFlag.gyro_ok = globalFlag.acc_ok = !( IIC_Write_1Byte(I2C_2, dev, reg, b) );
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
bitStart  Ŀ���ֽڵ���ʼλ
length   λ����
data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1
ʧ��Ϊ0
*******************************************************************************/
void IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{
    u8 b,mask;
    IIC_Read_nByte(I2C_2, dev, reg, 1, &b);
    mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
    data <<= (8 - length);
    data >>= (7 - bitStart);
    b &= mask;
    b |= data;
    IIC_Write_1Byte(I2C_2, dev, reg, b);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setSleepEnabled(uint8_t enabled)
*��������:	    ����  MPU6050 �Ƿ����˯��ģʽ
enabled =1   ˯��
enabled =0   ����
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) 
{
    IICwriteBit(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setClockSource(uint8_t source)
*��������:	    ����  MPU6050 ��ʱ��Դ
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source)
{
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:
*��������:	    ���� ������
*******************************************************************************/
void MPU6050_set_SMPLRT_DIV(uint16_t hz)
{
    IIC_Write_1Byte(I2C_2, MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV,1000/hz - 1);
    //I2C_Single_Write(MPU6050_ADDRESS,MPU_RA_SMPLRT_DIV, (1000/sample_rate - 1));
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*��������:	    ����  MPU6050 �����ǵ��������
*******************************************************************************/
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG,7, 3, 0x00);   //���Լ�
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*��������:	    ����  MPU6050 ���ٶȼƵ��������
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG,7, 3, 0x00);   //���Լ�
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:
*��������:	    ���õ�ͨ�˲���ֹƵ��
*******************************************************************************/
void MPU6050_setDLPF(uint8_t mode)
{
    IICwriteBits(MPU6050_ADDR, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(MPU6050_ADDR, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_initialize(void)
*��������:	    ��ʼ�� 	MPU6050 �Խ������״̬��
*******************************************************************************/
void MPU6050_Init(u16 lpf)
{
    u16 default_filter = 1;

    switch(lpf)
    {
    case 5:
        default_filter = MPU6050_DLPF_BW_5;
        break;
    case 10:
        default_filter = MPU6050_DLPF_BW_10;
        break;
    case 20:
        default_filter = MPU6050_DLPF_BW_20;
        break;
    case 42:
        default_filter = MPU6050_DLPF_BW_42;
        break;
    case 98:
        default_filter = MPU6050_DLPF_BW_98;
        break;
    case 188:
        default_filter = MPU6050_DLPF_BW_188;
        break;
    case 256:
        default_filter = MPU6050_DLPF_BW_256;
        break;
    default:
        default_filter = MPU6050_DLPF_BW_42;
        break;
    }
//    I2C_Soft_Init(I2C_2);

    //�豸��λ
//	IIC_Write_1Byte(MPU6050_ADDR,MPU6050_RA_PWR_MGMT_1, 0x80);

    MPU6050_setSleepEnabled(0); //���빤��״̬
    delay_ms(10);
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_ZGYRO); //����ʱ��  0x6b   0x03
    delay_ms(10);
    MPU6050_set_SMPLRT_DIV(1000);  //1000hz
    delay_ms(10);
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//������������� +-2000��ÿ��
    delay_ms(10);
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_8);	//���ٶȶ�������� +-8G
    delay_ms(10);
    MPU6050_setDLPF(default_filter);  //42hz
    delay_ms(10);
    MPU6050_setI2CMasterModeEnabled(0);	 //����MPU6050 ����AUXI2C
    delay_ms(10);
    MPU6050_setI2CBypassEnabled(1);	 //����������I2C��	MPU6050��AUXI2C	ֱͨ������������ֱ�ӷ���HMC5883L
    delay_ms(10);
}
