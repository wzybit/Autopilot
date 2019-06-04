
#ifndef IMU_H_
#define IMU_H_

#include "stm32f4xx.h"
#include "global.h"

typedef struct
{
	float center_pos_cm[VEC_XYZ];
	float gyro_rad[VEC_XYZ];
	float gyro_rad_old[VEC_XYZ];
	float gyro_rad_acc[VEC_XYZ];
	float linear_acc[VEC_XYZ];
}_center_pos_st;
extern _center_pos_st center_pos;

typedef struct 
{
	u8 acc_CALIBRATE;
	u8 gyr_CALIBRATE;
	u8 acc_z_auto_CALIBRATE;
	
	s16 Acc_Original[VEC_XYZ];
	s16 Gyro_Original[VEC_XYZ];
	
	float Acc[VEC_XYZ];//wzy ���ﻹ��float ��һ��
	float Acc_cmss[VEC_XYZ];
	float Gyro[VEC_XYZ];
	float Gyro_deg[VEC_XYZ];
	float Gyro_deg_lpf[VEC_XYZ];
	float Gyro_rad[VEC_XYZ];

	s16 Tempreature;
	float Tempreature_C;
	
}_sensor_st;//__attribute__((packed)) 

typedef struct
{
	float w;//q0;
	float x;//q1;
	float y;//q2;
	float z;//q3;

	float x_vec[VEC_XYZ]; //x���Ч��������
	float y_vec[VEC_XYZ]; //y���Ч��������
	float z_vec[VEC_XYZ]; //z���Ч�������� ����Ч��������
	float hx_vec[VEC_XYZ]; //�ں�����ת���õ��� ��ת����

	float a_acc[VEC_XYZ]; //��������ϵ �ں��õ��� ���ٶ�����
	float w_acc[VEC_XYZ]; //��������ϵ �ں��õ��� ���ٶ�����
	float h_acc[VEC_XYZ]; //��������ϵ �ں��õ��� ���ٶ�����
	
    uint8_t gravity_reset; //���� ������λ��� �����޸������ٶ�
    float vec_err[VEC_XYZ]; //�������������
    float vec_err_i[VEC_XYZ]; //����������������
    float gravity_kp;   //�������� kp
    float gravity_ki;   //�������� ki
    
    uint8_t mag_reset; //������ ���� ������λ��� �����޸������ٶ�
	float w_mag[VEC_XYZ]; //��������ϵ ��̬�����������������
    float mag_yaw_err;
    float mag_kp;   //�������� kp
    
	float roll; //���������̬��
	float pitch;
	float yaw;
} _imu_st ;

extern _sensor_st sensor;
extern _imu_st imu_data;

void IMU_Data_Prepare(u8 dT_ms);

void w2h_2d_trans(float w[VEC_XYZ],float ref_ax[VEC_XYZ],float h[VEC_XYZ]);
void h2w_2d_trans(float h[VEC_XYZ],float ref_ax[VEC_XYZ],float w[VEC_XYZ]);
void IMU_update(float dT, float gyro[VEC_XYZ], float acc[VEC_XYZ], float mag_val[VEC_XYZ], _imu_st *imu);

#endif //_IMU_H_
