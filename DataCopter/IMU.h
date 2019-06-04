
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
	
	float Acc[VEC_XYZ];//wzy 这里还是float 好一点
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

	float x_vec[VEC_XYZ]; //x轴等效方向向量
	float y_vec[VEC_XYZ]; //y轴等效方向向量
	float z_vec[VEC_XYZ]; //z轴等效方向向量 即等效重力向量
	float hx_vec[VEC_XYZ]; //融合坐标转换用到的 旋转向量

	float a_acc[VEC_XYZ]; //机体坐标系 融合用到的 加速度数据
	float w_acc[VEC_XYZ]; //世界坐标系 融合用到的 加速度数据
	float h_acc[VEC_XYZ]; //导航坐标系 融合用到的 加速度数据
	
    uint8_t gravity_reset; //重力 修正复位标记 用于修改修正速度
    float vec_err[VEC_XYZ]; //重力分量的误差
    float vec_err_i[VEC_XYZ]; //重力分量的误差积分
    float gravity_kp;   //重力修正 kp
    float gravity_ki;   //重力修正 ki
    
    uint8_t mag_reset; //磁力计 罗盘 修正复位标记 用于修改修正速度
	float w_mag[VEC_XYZ]; //世界坐标系 姿态解算所需磁力计数据
    float mag_yaw_err;
    float mag_kp;   //磁力修正 kp
    
	float roll; //解算出的姿态角
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
