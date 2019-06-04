
#include "Copter.h"

//�ǶȻ�PID������ʼ��
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
    
//���ٶȻ�PID������ʼ��
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
    
    //���ò���KI����
	copter.g.pid_attitude_angle[ROLL].set_kI(Parameter_save.value.pid_attitude_angle_save[ROLL][KI]);
    copter.g.pid_attitude_angle[PITCH].set_kI(Parameter_save.value.pid_attitude_angle_save[PITCH][KI]);
    
    /*����΢��*/
	target_angle_temp_roll  = - copter.ctrl_out_position[Y];
	target_angle_temp_pitch = - copter.ctrl_out_position[X];
	
	if(globalConfig.flightMode == STABILIZE) //wzy���ģʽҪע�⣬�����ǵ��·���ʱXY����Ư�Ƶ�ԭ��
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
	
	/*�����ο��Լ�������ο�����*/
	copter.ctrl_target_angle[ROLL]  = target_angle_temp_roll  + target_angle_adj_roll;
	copter.ctrl_target_angle[PITCH] = target_angle_temp_pitch + target_angle_adj_pitch;
	
	/*�����Ƕ��޷�*/
	copter.ctrl_target_angle[ROLL]  = LIMIT(copter.ctrl_target_angle[ROLL], -MAX_CTRL_ANGLE, MAX_CTRL_ANGLE);
	copter.ctrl_target_angle[PITCH] = LIMIT(copter.ctrl_target_angle[PITCH], -MAX_CTRL_ANGLE, MAX_CTRL_ANGLE);
	
	max_yaw_speed = 200;
	//
	/*ҡ����ת��ΪYAW�������ٶ�*/
	//set_yaw_av_tmp = (s32)(0.0023f *my_deadzone(copter.RC_CH[CH_YAW],0,65) *max_yaw_speed) + (-program_ctrl.yaw_pal_dps);//wzy
    set_yaw_av_tmp = (int32_t)(0.0023f * my_deadzone(copter.RC_CH[CH_YAW], 0, 65) * max_yaw_speed);

	/*���YAW���ٶ��޷�*/
	set_yaw_av_tmp = LIMIT(set_yaw_av_tmp , -max_yaw_speed, max_yaw_speed);
	
	/*û����ɣ���λ*/
	if(globalFlag.taking_off == 0)
	{
		copter.ctrl_target_angle[ROLL] = copter.ctrl_target_angle[PITCH] = 0;
        set_yaw_av_tmp = 0;
		copter.ctrl_target_angle[YAW] = copter.ctrl_feedback_angle[YAW];//yaw����Ϊ��ǰYAW�Ƕ�
	}
	/*�����������*/
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

	//�����޷�
	copter.set_yaw_speed += LIMIT((set_yaw_av_tmp - copter.set_yaw_speed), -30.0f, 30.0f);
	/*��������YAW�Ƕ�*/
	copter.ctrl_target_angle[YAW] += copter.set_yaw_speed * dT_s;
	/*����Ϊ+-180��*/
	if(copter.ctrl_target_angle[YAW] < -180.0f) copter.ctrl_target_angle[YAW] += 360.0f;
	else if(copter.ctrl_target_angle[YAW] > 180.0f) copter.ctrl_target_angle[YAW] -= 360.0f;	
	
	/*����YAW�Ƕ����*/
	copter.yaw_error = (copter.ctrl_target_angle[YAW] - copter.ctrl_feedback_angle[YAW]);
	/*����Ϊ+-180��*/
	if(copter.yaw_error < -180.0f) copter.yaw_error += 360.0f;
	else if(copter.yaw_error > 180.0f) copter.yaw_error -= 360.0f;

    /*��ֵ�����Ƕ�ֵ*/       
    copter.ctrl_feedback_angle[ROLL]  = imu_data.roll;
    copter.ctrl_feedback_angle[PITCH] = imu_data.pitch;
    copter.ctrl_feedback_angle[YAW]   = imu_data.yaw; 
			
	copter.g.pid_attitude_angle[ROLL].set_imax(5.0f * globalFlag.taking_off); //���ַ����޷�
    copter.g.pid_attitude_angle[PITCH].set_imax(5.0f * globalFlag.taking_off); //���ַ����޷�
    copter.g.pid_attitude_angle[YAW].set_imax(5.0f * globalFlag.taking_off); //���ַ����޷�
          
    copter.ctrl_out_angle[ROLL] = copter.g.pid_attitude_angle[ROLL].get_pid(dT_s,    //  ���ڣ���λ���룩 
                                                                copter.ctrl_target_rate[ROLL], //����ֵ���趨ֵ�� 
                                                                copter.ctrl_feedback_rate[ROLL]); //����ֵ����  
    copter.ctrl_out_angle[PITCH] = copter.g.pid_attitude_angle[PITCH].get_pid(dT_s,    //  ���ڣ���λ���룩 
                                                                copter.ctrl_target_rate[PITCH], //����ֵ���趨ֵ�� 
                                                                copter.ctrl_feedback_rate[PITCH]); //����ֵ����  
	copter.ctrl_out_angle[YAW] = copter.g.pid_attitude_angle[YAW].get_pid(dT_s,    //  ���ڣ���λ���룩 
                                                                copter.yaw_error, //����ֵ���趨ֵ�� 
                                                                0.0f); //����ֵ����  					
}

/*
��̬�����ʲ��ֿ��Ʋ���

kp���������ٶ���Ӧ�ٶȣ����𵴵�ǰ���£�����Խ��Խ�á�

���ԣ����Խ���kp������kd��

������kd�Ѿ����������𵴣���Ҫ��kp��kdͬʱ��С��
*/
void Copter::attitude_rate_control()
{
    float dT_s = 2e-3f; //2ms 
    
    //���ݷ���ģʽ ���ò���KI����
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
    
    /*Ŀ����ٶȸ�ֵ*/
     for(u8 i = 0;i<3;i++)
    {
        copter.ctrl_target_rate[i] = copter.ctrl_out_angle[i];
    }

    /*Ŀ����ٶ��޷�*/
    copter.ctrl_target_rate[ROLL]  = LIMIT(copter.ctrl_target_rate[ROLL], -1600.0f, 1600.0f);
    copter.ctrl_target_rate[PITCH] = LIMIT(copter.ctrl_target_rate[ROLL], -1600.0f, 1600.0f);

	/*�������ٶȸ�ֵ*/
	copter.ctrl_feedback_rate[ROLL]  += 0.5f * ( sensor.Gyro_deg_lpf[X] - copter.ctrl_feedback_rate[ROLL]);
	copter.ctrl_feedback_rate[PITCH] += 0.5f * ( sensor.Gyro_deg_lpf[X] - copter.ctrl_feedback_rate[PITCH]);
    copter.ctrl_feedback_rate[YAW]   += 0.5f * ( sensor.Gyro_deg_lpf[X] - copter.ctrl_feedback_rate[YAW]);

	/*PID����*/									 
    for(u8 i = 0; i < 3; i++)
    {
        copter.g.pid_attitude_rate[i].set_imax(250.0f * globalFlag.taking_off); //���ַ����޷�
          
        copter.ctrl_out_rate[i] = copter.g.pid_attitude_rate[i].get_pid(dT_s,    //  ���ڣ���λ���룩 
                                                                copter.ctrl_target_rate[i], //����ֵ���趨ֵ�� 
                                                                copter.ctrl_feedback_rate[i]); //����ֵ����                                  
    }
										 
	/*��ֵ�����ձ�������*/
	copter.motor_input[ROLL]  = (int16_t)(MOTOR_OUT_P * copter.ctrl_out_rate[ROLL]);
    copter.motor_input[PITCH] = (int16_t)(MOTOR_OUT_P * copter.ctrl_out_rate[PITCH]);
	copter.motor_input[YAW]   = (int16_t)(MOTOR_OUT_P * copter.ctrl_out_rate[YAW]);
	/*������޷�*/
	copter.motor_input[ROLL]  = LIMIT(copter.motor_input[ROLL], -1000, 1000);
	copter.motor_input[PITCH] = LIMIT(copter.motor_input[PITCH], -1000, 1000);
	copter.motor_input[YAW]   = LIMIT(copter.motor_input[YAW], -400, 400);	
}
