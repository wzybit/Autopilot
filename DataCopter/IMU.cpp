
#include "IMU.h"
#include "copter.h"

_center_pos_st center_pos;
_sensor_st sensor;

void motionless_check(u8 dT_ms) //静止检测 检查飞控有没有处于移动状态
{
	u8 t = 0;
    static s16 g_old[VEC_XYZ];
    static s16 g_d_sum[VEC_XYZ] = {500, 500, 500};
    
	for(u8 i = 0; i < 3; i++)
	{
		g_d_sum[i] += 3 * ABS(sensor.Gyro_Original[i] - g_old[i]) ;		
		g_d_sum[i] -= dT_ms ;			
		g_d_sum[i] = LIMIT(g_d_sum[i],0,200);		
        
		if( g_d_sum[i] > 10)
		{
			t++;
		}		
		g_old[i] = sensor.Gyro_Original[i];
	}

	if(t >= 2)
	{
		globalFlag.motionless = 0;	
	}
	else
	{
		globalFlag.motionless = 1;
	}
}

void IMU_Data_Offset()
{
	static u8 off_cnt;
    static u16 acc_z_auto_cnt = 0, acc_sum_cnt = 0, gyro_sum_cnt = 0;
    static s32 sum_temp[7]={0,0,0,0,0,0,0};
    static s32 acc_auto_sum_temp[3];
    static s16 acc_z_auto[4];
    
    if(sensor.acc_z_auto_CALIBRATE)//这里的acc z轴加一次单独校准
	{
		acc_z_auto_cnt++;	
        for(u8 i = 0;i<3;i++)
        {
            acc_auto_sum_temp[A_X + i] += sensor.Acc_Original[i]; //wzy这里之前单独校准z轴时 用的就是原始数据
        }        
		
		if(acc_z_auto_cnt>=OFFSET_AV_NUM)
		{
			sensor.acc_z_auto_CALIBRATE = 0;
			acc_z_auto_cnt = 0;
			for(u8 i = 0;i<3;i++)
			{			
				acc_z_auto[i] = acc_auto_sum_temp[i]/OFFSET_AV_NUM;
				
				acc_auto_sum_temp[i] = 0;
			}			
			acc_z_auto[3] = my_sqrt( 4096*4096 - (my_pow(acc_z_auto[0]) + my_pow(acc_z_auto[1])) );			
			globalSaveData.acc_offset[Z] = acc_z_auto[2] - acc_z_auto[3];	
		}		
	}
    
	if(sensor.gyr_CALIBRATE || sensor.acc_CALIBRATE || sensor.acc_z_auto_CALIBRATE)
	{	
        //复位校准值	//在有移动，翻转或者传感器加热没有初始化的情况下，不进行校准
		if(globalFlag.motionless == 0 || sensor.Acc_Original[Z]<1000 || (globalFlag.mems_temperature_ok == 0))
		{
            gyro_sum_cnt = 0;
            acc_sum_cnt=0;
            acc_z_auto_cnt = 0;
            
            for(u8 j=0;j<3;j++)
            {
                acc_auto_sum_temp[j] = sum_temp[G_X+j] = sum_temp[A_X+j] = 0;
            }
            sum_temp[TEM] = 0;
		}

		off_cnt++;			
		if(off_cnt>=10)
		{	
			off_cnt=0;

			if(sensor.gyr_CALIBRATE)
			{
				LED_STA.calGyr = 1;
				gyro_sum_cnt++;				
				for(u8 i = 0;i<3;i++)
				{
					sum_temp[G_X+i] += sensor.Gyro_Original[i];
				}
				if( gyro_sum_cnt >= OFFSET_AV_NUM )
				{
					for(u8 i = 0;i<3;i++)
					{
						globalSaveData.gyro_offset[i] = sum_temp[G_X+i]/OFFSET_AV_NUM;//wzy 原版这里突然用float不合适吧						
						sum_temp[G_X + i] = 0;
					}
					gyro_sum_cnt =0;
					if(sensor.gyr_CALIBRATE == 1)
					{
						if(sensor.acc_CALIBRATE == 0)
						{
							data_save();
						}
					}
					sensor.gyr_CALIBRATE = 0;
					GCS_SendString("GYR init OK!");
					LED_STA.calGyr = 0;
				}
			}
			
			if(sensor.acc_CALIBRATE == 1)
			{
				LED_STA.calAcc = 1;
				acc_sum_cnt++;		
				sum_temp[A_X] += sensor.Acc_Original[0];//wzy这里之前校准加速度计时，用的就是原始数据
				sum_temp[A_Y] += sensor.Acc_Original[1];
				sum_temp[A_Z] += sensor.Acc_Original[2] - 4096;// - 65535/16;   // +-8G
				sum_temp[TEM] += sensor.Tempreature;
				if( acc_sum_cnt >= OFFSET_AV_NUM )
				{
					for(u8 i=0 ;i<3;i++)
					{
						globalSaveData.acc_offset[i] = sum_temp[A_X+i]/OFFSET_AV_NUM;						
						sum_temp[A_X + i] = 0;
					}
					acc_sum_cnt =0;
					sensor.acc_CALIBRATE = 0;
					GCS_SendString("ACC init OK!");
					LED_STA.calAcc = 0;
					data_save();
				}	
			}
		}
	}
}

static float gyr_f1[VEC_XYZ],acc_f1[VEC_XYZ];

void IMU_Data_Prepare(u8 dT_ms)
{	
	float hz = 0 ;
	if(dT_ms != 0) hz = 1000/dT_ms;
	
    static s32 sensor_val[6];
    static s32 sensor_val_rot[6];
    static s32 sensor_val_ref[6];
	/*静止检测*/
	motionless_check(dT_ms);
			
	IMU_Data_Offset(); //校准函数

	/*读取buffer原始数据*/            //wzy 为什么 ACC 要 <<1
	sensor.Acc_Original[X] = (s16)((((s16)imu_mpu6050_buffer[0]) << 8) | imu_mpu6050_buffer[1])<<1;//>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
	sensor.Acc_Original[Y] = (s16)((((s16)imu_mpu6050_buffer[2]) << 8) | imu_mpu6050_buffer[3])<<1;//>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
	sensor.Acc_Original[Z] = (s16)((((s16)imu_mpu6050_buffer[4]) << 8) | imu_mpu6050_buffer[5])<<1;//>>1;// + 4 *sensor.Tempreature_C;// + 7 *sensor.Tempreature_C;
 
	sensor.Gyro_Original[X] = (s16)((((s16)imu_mpu6050_buffer[ 8]) << 8) | imu_mpu6050_buffer[ 9]) ;
	sensor.Gyro_Original[Y] = (s16)((((s16)imu_mpu6050_buffer[10]) << 8) | imu_mpu6050_buffer[11]) ;
	sensor.Gyro_Original[Z] = (s16)((((s16)imu_mpu6050_buffer[12]) << 8) | imu_mpu6050_buffer[13]) ;

	sensor.Tempreature = ((((int16_t)imu_mpu6050_buffer[6]) << 8) | imu_mpu6050_buffer[7]); //tempreature
	//imu温度
	sensor.Tempreature_C = sensor.Tempreature/326.8f + 25 ;//sensor.Tempreature/340.0f + 36.5f;
	
	//得出校准后的数据 //wzy 加速度计就在这减偏移量吧 原版在下面坐标转换90度处转 没有区别
	for(u8 i=0;i<3;i++)
	{ 		
		sensor_val[A_X+i] = sensor.Acc_Original[i] - globalSaveData.acc_offset[i];
		sensor_val[G_X+i] = sensor.Gyro_Original[i] - globalSaveData.gyro_offset[i] ;
	}	

	/*赋值*/
	for(u8 i = 0;i<6;i++)
	{
		sensor_val_rot[i] = sensor_val[i];
	}

	// 数据坐标转90度//wzy 这里怎么转 看机头朝向 
	sensor_val_ref[G_X] =  sensor_val_rot[G_X];
	sensor_val_ref[G_Y] = -sensor_val_rot[G_Y];
	sensor_val_ref[G_Z] =  sensor_val_rot[G_Z];
	
	sensor_val_ref[A_X] =  sensor_val_rot[A_X];
	sensor_val_ref[A_Y] = -sensor_val_rot[A_Y];
	sensor_val_ref[A_Z] =  sensor_val_rot[A_Z];

	//软件滤波
	for(u8 i=0;i<3;i++)
	{	
		//0.24f，1ms ，50hz截止; 0.15f,1ms,28hz; 0.1f,1ms,18hz
		gyr_f1[X + i] += GYRO_ACC_FILTER * (sensor_val_ref[G_X + i] - sensor.Gyro[X + i]);
		acc_f1[X + i] += GYRO_ACC_FILTER * (sensor_val_ref[A_X + i] - sensor.Acc[X + i]);				
	}
	
	//旋转加速度补偿
	for(u8 i = 0; i < 3; i++)
	{	
		center_pos.gyro_rad_old[i] = center_pos.gyro_rad[i];
		center_pos.gyro_rad[i] =  gyr_f1[X + i] * RANGE_PN2000_TO_RAD;//0.001065f;
		center_pos.gyro_rad_acc[i] = hz * (center_pos.gyro_rad[i] - center_pos.gyro_rad_old[i]);
	}
	center_pos.linear_acc[X] = +center_pos.gyro_rad_acc[Z] * center_pos.center_pos_cm[Y] - center_pos.gyro_rad_acc[Y] * center_pos.center_pos_cm[Z];
	center_pos.linear_acc[Y] = -center_pos.gyro_rad_acc[Z] * center_pos.center_pos_cm[X] + center_pos.gyro_rad_acc[X] * center_pos.center_pos_cm[Z];
	center_pos.linear_acc[Z] = +center_pos.gyro_rad_acc[Y] * center_pos.center_pos_cm[X] - center_pos.gyro_rad_acc[X] * center_pos.center_pos_cm[Y];
	
	for(u8 i=0;i<3;i++)
	{
		sensor.Gyro[X+i] = gyr_f1[i];//sensor_val_ref[G_X + i];
		sensor.Acc[X+i] = acc_f1[i] - center_pos.linear_acc[i]/RANGE_PN8G_TO_CMSS;//sensor_val_ref[A_X+i];//
        //sensor.Acc[X+i] = acc_f1[i];//sensor_val_ref[A_X+i];//wzy
	}
	
	//转换单位
    for(u8 i =0 ;i<3;i++)
    {
        /*陀螺仪转换到度每秒，量程+-2000度*/
        sensor.Gyro_deg[i] = sensor.Gyro[i] *0.061036f ;//  /65535 * 4000; +-2000度 0.061
        /*陀螺仪再低通一次*/
        sensor.Gyro_deg_lpf[i] += GYRO_ACC_FILTER *(sensor.Gyro_deg[i] - sensor.Gyro_deg_lpf[i]);
        /*陀螺仪转换到弧度度每秒，量程+-2000度*/
        sensor.Gyro_rad[i] = sensor.Gyro_deg_lpf[i] *0.01745f;//sensor.Gyro[i] *RANGE_PN2000_TO_RAD ;//  0.001065264436f //微调值 0.0010652f
        /*加速度计转换到厘米每平方秒，量程+-8G*/
        sensor.Acc_cmss[i] = sensor.Acc[i] * RANGE_PN8G_TO_CMSS;//   /65535 * 16*981; +-8G		
    }
}

void Copter::Center_Pos_Set(void)
{
	center_pos.center_pos_cm[X] = globalSaveData.center_pos_cm[X];  //+0.0f;
	center_pos.center_pos_cm[Y] = globalSaveData.center_pos_cm[Y];  //-0.0f;
	center_pos.center_pos_cm[Z] = globalSaveData.center_pos_cm[Z];  //+0.0f;
}


/**************至此惯性传感器读取及处理结束 以下为姿态解算部分***************/

/*参考坐标

俯视，机头方向为x正方向
     +x
     |
 +y--|--
     |
		 
*/	

//世界坐标平面XY转平面航向坐标XY
void w2h_2d_trans(float w[VEC_XYZ],float ref_ax[VEC_XYZ],float h[VEC_XYZ])
{
	h[X] =  w[X] *  ref_ax[X]  + w[Y] *ref_ax[Y];
	h[Y] =  w[X] *(-ref_ax[Y]) + w[Y] *ref_ax[X];	
}
//平面航向坐标XY转世界坐标平面XY
void h2w_2d_trans(float h[VEC_XYZ],float ref_ax[VEC_XYZ],float w[VEC_XYZ])
{
	w[X] = h[X] *ref_ax[X] + h[Y] *(-ref_ax[Y]);
	w[Y] = h[X] *ref_ax[Y] + h[Y] *  ref_ax[X];	
}

_imu_st imu_data =  {1,0,0,0}; //wzy 这里还有个初始化的问题要注意 gravity等的初始化
void IMU_update(float dT, float gyro[VEC_XYZ], float acc[VEC_XYZ], float mag_val[VEC_XYZ], _imu_st *imu)
{    
    static float q0q1, q0q2, q1q1, q1q3, q2q2, q2q3, q3q3, q1q2, q0q3;//四元数计算
    static float attitude_matrix[3][3]; //必须由姿态解算算出该矩阵
    static float acc_norm_length, acc_norm[VEC_XYZ]; //加速度向量归一化
    static float mag_err_dot_prudoct; //磁力计数据
    static float mag_2d_w_vec[2][2] = {{1,0},{1,0}}; //地理坐标中，水平面磁场方向恒为南北 (1,0)
    static float imu_gravity_reset_val; //重力复位修正判断值
    static u16 gravity_reset_cnt; //重力复位修正判断计数
    float d_angle[VEC_XYZ]; //修正后的角速度向量
    float q_norm_length; //四元数归一化
    static float t_temp; //姿态角计算临时使用
    
    q0q1 = imu->w * imu->x;
    q0q2 = imu->w * imu->y;
    q1q1 = imu->x * imu->x;
    q1q3 = imu->x * imu->z;
    q2q2 = imu->y * imu->y;
    q2q3 = imu->y * imu->z;
    q3q3 = imu->z * imu->z;
    q1q2 = imu->x * imu->y;
    q0q3 = imu->w * imu->z;
    
    // 载体坐标下的x方向向量，单位化。
    attitude_matrix[0][0] = imu->x_vec[X] = 1.0f - (2.0f * q2q2 + 2.0f * q3q3);
    attitude_matrix[0][1] = imu->x_vec[Y] = 2.0f * q1q2 - 2.0f * q0q3;
    attitude_matrix[0][2] = imu->x_vec[Z] = 2.0f * q1q3 + 2.0f * q0q2;
		
	// 载体坐标下的y方向向量，单位化。
    attitude_matrix[1][0] = imu->y_vec[X] = 2.0f * q1q2 + 2.0f * q0q3;
    attitude_matrix[1][1] = imu->y_vec[Y] = 1.0f - (2.0f * q1q1 + 2.0f * q3q3);
    attitude_matrix[1][2] = imu->y_vec[Z] = 2.0f * q2q3 - 2.0f * q0q1;
		
    // 载体坐标下的z方向向量（等效重力向量、重力加速度向量），单位化。
    attitude_matrix[2][0] = imu->z_vec[X] = 2.0f * q1q3 - 2.0f * q0q2;
    attitude_matrix[2][1] = imu->z_vec[Y] = 2.0f * q2q3 + 2.0f * q0q1;
    attitude_matrix[2][2] = imu->z_vec[Z] = 1.0f - (2.0f * q1q1 + 2.0f * q2q2);

//===============================================================
// 计算载体坐标下的运动加速度。(与姿态解算无关，这里的计算用于加速度计与其它传感器融合使用)
    float hx_vec_reci = my_sqrt_reciprocal(my_pow(attitude_matrix[0][0]) + my_pow(attitude_matrix[1][0]));
	imu->hx_vec[X] = attitude_matrix[0][0] * hx_vec_reci;
	imu->hx_vec[Y] = attitude_matrix[1][0] * hx_vec_reci;
    for(u8 i = 0; i < 3; i++)
    {
        imu->a_acc[i] = acc[i] - 981.0f * imu->z_vec[i];//wzy 981 是 100G ? 这个a_acc需要观测一下
    }
    //计算世界坐标下的运动加速度。坐标系为北西天
    for(u8 i = 0;i<3;i++)
    {
        float temp = 0.0f;
        for(u8 j = 0;j < 3; j++)
        {            
            temp += imu->a_acc[j] * attitude_matrix[i][j];
        }
        imu->w_acc[i] = temp;
    }
    w2h_2d_trans(imu->w_acc, imu->hx_vec, imu->h_acc);
  
//============加速度计数据引入 修正向量误差========    
    //如果加速计各轴的数均是0，那么忽略该加速度数据。否则在加速计数据归一化处理的时候，会导致除以0的错误。
    //if(!(acc[X] == 0.0f && acc[Y] == 0.0f && acc[Z] == 0.0f))
	{
        acc_norm_length = my_sqrt(acc[X] * acc[X] + acc[Y] * acc[Y] + acc[Z] * acc[Z]);
        for(u8 i = 0; i < 3; i++)
        {
            acc_norm[i] = acc[i] / acc_norm_length;        // 加速度计的读数，单位化。
        }        
        // 测量值与等效重力向量的叉积（计算向量误差）。
        imu->vec_err[X] =  (acc_norm[Y] * imu->z_vec[Z] - imu->z_vec[Y] * acc_norm[Z]);
        imu->vec_err[Y] = -(acc_norm[X] * imu->z_vec[Z] - imu->z_vec[X] * acc_norm[Z]);
        imu->vec_err[Z] = -(acc_norm[Y] * imu->z_vec[X] - imu->z_vec[Y] * acc_norm[X]); 
        if(acc_norm_length > 1060.0f || acc_norm_length < 900.0f)//wzy这个条件还要注意，可能导致重力无法修正 量程+-8G
        {
            imu->vec_err[X] = imu->vec_err[Y] = imu->vec_err[Z] = 0;
        }        
        for(u8 i = 0; i < 3; i++)
        {
            imu->vec_err_i[i] +=  LIMIT(imu->vec_err[i], -0.1f, 0.1f) * dT * imu->gravity_ki; //误差积分
        }
    }
    
//============磁力计数据引入 修正向量误差========
    if(!(mag_val[X] == 0.0f && mag_val[Y] == 0.0f && mag_val[Z] == 0.0f))
    {
        //把载体坐标下的罗盘数据转换到地理坐标下
        for(u8 i = 0;i<3;i++)
		{
			float temp = 0;
			for(u8 j = 0;j<3;j++)
			{				
				temp += mag_val[j] * attitude_matrix[i][j];
			}
			imu->w_mag[i] = temp;
		}
        //计算方向向量归一化系数（模的倒数）
        float l_re_tmp = my_sqrt_reciprocal(my_pow(imu->w_mag[0]) + my_pow(imu->w_mag[1]));
        //计算南北朝向向量
        mag_2d_w_vec[1][0] = imu->w_mag[0] * l_re_tmp;
        mag_2d_w_vec[1][1] = imu->w_mag[1] * l_re_tmp;
        //计算南北朝向误差(叉乘)，地理坐标中，水平面磁场方向向量应恒为南北 (1,0)
        imu->mag_yaw_err = vec_2_cross_product(mag_2d_w_vec[1],mag_2d_w_vec[0]);
        //计算南北朝向向量点乘，判断同向或反向
        mag_err_dot_prudoct = vec_2_dot_product(mag_2d_w_vec[1],mag_2d_w_vec[0]);
        //若反向，直接给最大误差
        if(mag_err_dot_prudoct<0)
        {
            imu->mag_yaw_err = my_sign(imu->mag_yaw_err) * 1.0f;
        }		
    }
    
//==================修正开关=====================
    if(globalFlag.mag_ok == 0)//判断有无使用磁力计
    {
        imu->mag_kp = 0; //不修正
        imu->mag_reset = 0; //罗盘修正不复位，清除复位标记
    }
    else
    {
        if(imu->mag_reset)//
        {            
            imu->mag_kp = 10.0f; //通过增量进行对准
            if(imu->mag_yaw_err != 0.0f && ABS(imu->mag_yaw_err) < 0.02f)
            {
                imu->mag_reset = 0;//误差小于2的时候，清除复位标记
            }
        }
        else
        {
            imu->mag_kp = 0.1f; //正常修正
        }
    }

    if(imu->gravity_reset == 0)//没有复位标志，重力正常修正
    {			
        imu->gravity_kp = 0.2f;
        imu->gravity_ki = 0.01f;
    }
    else//快速修正，通过增量进行对准
    {
        imu->gravity_kp = 10.0f;
        imu->gravity_ki = 0.0f;       
        imu_gravity_reset_val = ABS(imu->vec_err[X]) + ABS(imu->vec_err[Y]); //计算静态误差是否缩小        
        imu_gravity_reset_val = LIMIT(imu_gravity_reset_val, 0.0f, 1.0f);
        if((imu_gravity_reset_val < 0.05f) && (imu->mag_reset == 0))
        {            
            gravity_reset_cnt += 2;//计时
            if(gravity_reset_cnt>400)
            {
                gravity_reset_cnt = 0;
                imu->gravity_reset = 0;//已经对准，清除复位标记
            }
        }
        else
        {
            gravity_reset_cnt = 0;
        }
    }		

    //角速度向量 融合加速度计和磁力计误差向量
    for(u8 i = 0; i < 3; i++)
	{        
        d_angle[i] = dT / 2.0f * (gyro[i] + imu->vec_err[i] * imu->gravity_kp + imu->vec_err_i[i] + 
                    imu->mag_yaw_err * imu->z_vec[i] * imu->mag_kp);
	}
    //四元数更新及归一化
    imu->w = imu->w              - imu->x * d_angle[X] - imu->y * d_angle[Y] - imu->z * d_angle[Z];
    imu->x = imu->w * d_angle[X] + imu->x              + imu->y * d_angle[Z] - imu->z * d_angle[Y];
    imu->y = imu->w * d_angle[Y] - imu->x * d_angle[Z] + imu->y              + imu->z * d_angle[X];
    imu->z = imu->w * d_angle[Z] + imu->x * d_angle[Y] - imu->y * d_angle[X] + imu->z;		
	q_norm_length = my_sqrt_reciprocal(imu->w * imu->w + imu->x * imu->x + imu->y * imu->y + imu->z * imu->z);
    imu->w *= q_norm_length;
    imu->x *= q_norm_length;
    imu->y *= q_norm_length;
    imu->z *= q_norm_length;
        		
    //计算姿态角
	t_temp = LIMIT(1.0f - my_pow(attitude_matrix[2][0]), 0.0f, 1.0f);
    if(ABS(imu_data.z_vec[Z]) > 0.05f)//避免奇点的运算
    {
        imu_data.pitch =  fast_atan2(attitude_matrix[2][0], my_sqrt(t_temp)) * 57.30f;
        imu_data.roll  =  fast_atan2(attitude_matrix[2][1], attitude_matrix[2][2]) * 57.30f; 
        imu_data.yaw   = -fast_atan2(attitude_matrix[1][0], attitude_matrix[0][0]) * 57.30f; 
    }
}
