
#include "Copter.h"

//角度环PID参数初始化
void Copter::ctrl_pid_angle_set(void)
{
	g.pid_attitude_angle[ROLL].set_kP(Parameter_save.value.pid_attitude_angle_save[ROLL][KP]);
    g.pid_attitude_angle[ROLL].set_kI(Parameter_save.value.pid_attitude_angle_save[ROLL][KI]);
    g.pid_attitude_angle[ROLL].set_kd_expect(0.0f);
    g.pid_attitude_angle[ROLL].set_kD(Parameter_save.value.pid_attitude_angle_save[ROLL][KD]);
	
    g.pid_attitude_angle[PITCH].set_kP(Parameter_save.value.pid_attitude_angle_save[PITCH][KP]);
    g.pid_attitude_angle[PITCH].set_kI(Parameter_save.value.pid_attitude_angle_save[PITCH][KI]);
    g.pid_attitude_angle[PITCH].set_kd_expect(0.0f);
    g.pid_attitude_angle[PITCH].set_kD(Parameter_save.value.pid_attitude_angle_save[PITCH][KD]);
    
    g.pid_attitude_angle[YAW].set_kP(Parameter_save.value.pid_attitude_angle_save[YAW][KP]);
    g.pid_attitude_angle[YAW].set_kI(Parameter_save.value.pid_attitude_angle_save[YAW][KI]);
    g.pid_attitude_angle[YAW].set_kd_expect(Parameter_save.value.pid_attitude_angle_save[YAW][KD]);
    g.pid_attitude_angle[YAW].set_kD(Parameter_save.value.pid_attitude_angle_save[YAW][KD]);
}
    
//角速度环PID参数初始化
void Copter::ctrl_pid_rate_set(void)
{
	g.pid_attitude_rate[ROLL].set_kP(Parameter_save.value.pid_attitude_rate_save[ROLL][KP]);
    g.pid_attitude_rate[ROLL].set_kI(Parameter_save.value.pid_attitude_rate_save[ROLL][KI]);
    g.pid_attitude_rate[ROLL].set_kd_expect(Parameter_save.value.pid_attitude_rate_save[ROLL][KD]);
    g.pid_attitude_rate[ROLL].set_kD(Parameter_save.value.pid_attitude_rate_save[ROLL][KD]);
	
    g.pid_attitude_rate[PITCH].set_kP(Parameter_save.value.pid_attitude_rate_save[PITCH][KP]);
    g.pid_attitude_rate[PITCH].set_kI(Parameter_save.value.pid_attitude_rate_save[PITCH][KI]);
    g.pid_attitude_rate[PITCH].set_kd_expect(Parameter_save.value.pid_attitude_rate_save[PITCH][KD]);
    g.pid_attitude_rate[PITCH].set_kD(Parameter_save.value.pid_attitude_rate_save[PITCH][KD]);
    
    g.pid_attitude_rate[YAW].set_kP(Parameter_save.value.pid_attitude_rate_save[YAW][KP]);
    g.pid_attitude_rate[YAW].set_kI(Parameter_save.value.pid_attitude_rate_save[YAW][KI]);
    g.pid_attitude_rate[YAW].set_kd_expect(Parameter_save.value.pid_attitude_rate_save[YAW][KD]);
    g.pid_attitude_rate[YAW].set_kD(Parameter_save.value.pid_attitude_rate_save[YAW][KD]);
	
#if (MOTOR_ESC_TYPE == 2)
    g.pid_attitude_rate[ROLL].set_kD(Parameter_save.value.pid_attitude_rate_save[ROLL][KD] * 0.3f);
	g.pid_attitude_rate[PITCH].set_kD(Parameter_save.value.pid_attitude_rate_save[PITCH][KD] * 0.3f);
#elif (MOTOR_ESC_TYPE == 1)
	g.pid_attitude_rate[ROLL].set_kD(Parameter_save.value.pid_attitude_rate_save[ROLL][KD] * 1.0f);
	g.pid_attitude_rate[PITCH].set_kD(Parameter_save.value.pid_attitude_rate_save[PITCH][KD] * 1.0f);
#endif
}

void Copter::attitude_angle_control()
{
    float dT_s = 5e-3f; //5ms
    static float target_angle_temp_roll,target_angle_temp_pitch;
    static float target_angle_adj_roll,target_angle_adj_pitch;
    int32_t max_yaw_speed;
    static int32_t set_yaw_av_tmp;
    
    //设置部分KI参数
	copter.g.pid_attitude_angle[ROLL].set_kI(Parameter_save.value.pid_attitude_angle_save[ROLL][KI]);
    copter.g.pid_attitude_angle[PITCH].set_kI(Parameter_save.value.pid_attitude_angle_save[PITCH][KI]);
    
    /*积分微调*/
	target_angle_temp_roll  = - copter.ctrl_out_position[Y];
	target_angle_temp_pitch = - copter.ctrl_out_position[X];
	
	if(globalConfig.flightMode == STABILIZE) //wzy这个模式要注意，可能是导致飞行时XY方向漂移的原因
	{
		if(ABS(target_angle_temp_roll + target_angle_adj_roll) < 5)
		{
			target_angle_adj_roll += 0.2f * target_angle_temp_roll * dT_s;
			target_angle_adj_roll  = LIMIT(target_angle_adj_roll, -1.0f, 1.0f);
		}
		
		if(ABS(target_angle_temp_pitch + target_angle_adj_pitch) < 5)
		{
			target_angle_adj_pitch += 0.2f *target_angle_temp_pitch *dT_s;
			target_angle_adj_pitch  = LIMIT(target_angle_adj_pitch, -1.0f, 1.0f);
		}
	}
	else
	{
		target_angle_adj_roll  = 0.0f;
		target_angle_adj_pitch = 0.0f;
	}
	
	/*正负参考自驾仪坐标参考方向*/
	copter.ctrl_target_angle[ROLL]  = target_angle_temp_roll  + target_angle_adj_roll;
	copter.ctrl_target_angle[PITCH] = target_angle_temp_pitch + target_angle_adj_pitch;
	
	/*期望角度限幅*/
	copter.ctrl_target_angle[ROLL]  = LIMIT(copter.ctrl_target_angle[ROLL], -MAX_CTRL_ANGLE, MAX_CTRL_ANGLE);
	copter.ctrl_target_angle[PITCH] = LIMIT(copter.ctrl_target_angle[PITCH], -MAX_CTRL_ANGLE, MAX_CTRL_ANGLE);
	
	max_yaw_speed = 200;
	//
	/*摇杆量转换为YAW期望角速度*/
	//set_yaw_av_tmp = (s32)(0.0023f *my_deadzone(copter.RC_CH[CH_YAW],0,65) *max_yaw_speed) + (-program_ctrl.yaw_pal_dps);//wzy
    set_yaw_av_tmp = (int32_t)(0.0023f * my_deadzone(copter.RC_CH[CH_YAW], 0, 65) * max_yaw_speed);

	/*最大YAW角速度限幅*/
	set_yaw_av_tmp = LIMIT(set_yaw_av_tmp , -max_yaw_speed, max_yaw_speed);
	
	/*没有起飞，复位*/
	if(globalFlag.taking_off == 0)
	{
		copter.ctrl_target_angle[ROLL] = copter.ctrl_target_angle[PITCH] = 0;
        set_yaw_av_tmp = 0;
		copter.ctrl_target_angle[YAW] = copter.ctrl_feedback_angle[YAW];//yaw重置为当前YAW角度
	}
	/*限制误差增大*/
	if(copter.yaw_error > 90.0f)
	{
		if(set_yaw_av_tmp>0)
		{
			set_yaw_av_tmp = 0;
		}
	}
	else if(copter.yaw_error < -90.0f)
	{
		if(set_yaw_av_tmp<0)
		{
			set_yaw_av_tmp = 0;
		}
	}	

	//增量限幅
	copter.set_yaw_speed += LIMIT((set_yaw_av_tmp - copter.set_yaw_speed), -30.0f, 30.0f);
	/*设置期望YAW角度*/
	copter.ctrl_target_angle[YAW] += copter.set_yaw_speed * dT_s;
	/*限制为+-180度*/
	if(copter.ctrl_target_angle[YAW] < -180.0f) copter.ctrl_target_angle[YAW] += 360.0f;
	else if(copter.ctrl_target_angle[YAW] > 180.0f) copter.ctrl_target_angle[YAW] -= 360.0f;	
	
	/*计算YAW角度误差*/
	copter.yaw_error = (copter.ctrl_target_angle[YAW] - copter.ctrl_feedback_angle[YAW]);
	/*限制为+-180度*/
	if(copter.yaw_error < -180.0f) copter.yaw_error += 360.0f;
	else if(copter.yaw_error > 180.0f) copter.yaw_error -= 360.0f;

    /*赋值反馈角度值*/       
    copter.ctrl_feedback_angle[ROLL]  = imu_data.roll;
    copter.ctrl_feedback_angle[PITCH] = imu_data.pitch;
    copter.ctrl_feedback_angle[YAW]   = imu_data.yaw; 
			
	copter.g.pid_attitude_angle[ROLL].set_imax(5.0f * globalFlag.taking_off); //积分幅度限幅
    copter.g.pid_attitude_angle[PITCH].set_imax(5.0f * globalFlag.taking_off); //积分幅度限幅
    copter.g.pid_attitude_angle[YAW].set_imax(5.0f * globalFlag.taking_off); //积分幅度限幅
          
    copter.ctrl_out_angle[ROLL] = copter.g.pid_attitude_angle[ROLL].get_pid(dT_s,    //  周期（单位：秒） 
                                                                copter.ctrl_target_rate[ROLL], //期望值（设定值） 
                                                                copter.ctrl_feedback_rate[ROLL]); //反馈值（）  
    copter.ctrl_out_angle[PITCH] = copter.g.pid_attitude_angle[PITCH].get_pid(dT_s,    //  周期（单位：秒） 
                                                                copter.ctrl_target_rate[PITCH], //期望值（设定值） 
                                                                copter.ctrl_feedback_rate[PITCH]); //反馈值（）  
	copter.ctrl_out_angle[YAW] = copter.g.pid_attitude_angle[YAW].get_pid(dT_s,    //  周期（单位：秒） 
                                                                copter.yaw_error, //期望值（设定值） 
                                                                0.0f); //反馈值（）  					
}

/*
姿态角速率部分控制参数

kp：调整角速度响应速度，不震荡的前提下，尽量越高越好。

震荡试，可以降低kp，增大kd。

若增大kd已经不能抑制震荡，需要将kp和kd同时减小。
*/
void Copter::attitude_rate_control()
{
    float dT_s = 2e-3f; //2ms 
    
    //根据飞行模式 设置部分KI参数
	if(globalFlag.auto_take_off_land == AUTO_TAKE_OFF)
    {
        copter.g.pid_attitude_rate[ROLL].set_kI(0.0f);
        copter.g.pid_attitude_rate[PITCH].set_kI(0.0f);
    }
    else
    {
        copter.g.pid_attitude_rate[ROLL].set_kI(Parameter_save.value.pid_attitude_rate_save[ROLL][KI]);
        copter.g.pid_attitude_rate[PITCH].set_kI(Parameter_save.value.pid_attitude_rate_save[PITCH][KI]);
    }	
    
    /*目标角速度赋值*/
     for(u8 i = 0;i<3;i++)
    {
        copter.ctrl_target_rate[i] = copter.ctrl_out_angle[i];
    }

    /*目标角速度限幅*/
    copter.ctrl_target_rate[ROLL]  = LIMIT(copter.ctrl_target_rate[ROLL], -1600.0f, 1600.0f);
    copter.ctrl_target_rate[PITCH] = LIMIT(copter.ctrl_target_rate[ROLL], -1600.0f, 1600.0f);

	/*反馈角速度赋值*/
	copter.ctrl_feedback_rate[ROLL]  += 0.5f * ( sensor.Gyro_deg_lpf[X] - copter.ctrl_feedback_rate[ROLL]);
	copter.ctrl_feedback_rate[PITCH] += 0.5f * ( sensor.Gyro_deg_lpf[X] - copter.ctrl_feedback_rate[PITCH]);
    copter.ctrl_feedback_rate[YAW]   += 0.5f * ( sensor.Gyro_deg_lpf[X] - copter.ctrl_feedback_rate[YAW]);

	/*PID计算*/									 
    for(u8 i = 0; i < 3; i++)
    {
        copter.g.pid_attitude_rate[i].set_imax(250.0f * globalFlag.taking_off); //积分幅度限幅
          
        copter.ctrl_out_rate[i] = copter.g.pid_attitude_rate[i].get_pid(dT_s,    //  周期（单位：秒） 
                                                                copter.ctrl_target_rate[i], //期望值（设定值） 
                                                                copter.ctrl_feedback_rate[i]); //反馈值（）                                  
    }
										 
	/*赋值，最终比例调节*/
	copter.motor_input[ROLL]  = (int16_t)(MOTOR_OUT_P * copter.ctrl_out_rate[ROLL]);
    copter.motor_input[PITCH] = (int16_t)(MOTOR_OUT_P * copter.ctrl_out_rate[PITCH]);
	copter.motor_input[YAW]   = (int16_t)(MOTOR_OUT_P * copter.ctrl_out_rate[YAW]);
	/*输出量限幅*/
	copter.motor_input[ROLL]  = LIMIT(copter.motor_input[ROLL], -1000, 1000);
	copter.motor_input[PITCH] = LIMIT(copter.motor_input[PITCH], -1000, 1000);
	copter.motor_input[YAW]   = LIMIT(copter.motor_input[YAW], -400, 400);	
}
