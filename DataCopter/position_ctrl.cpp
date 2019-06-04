
#include "Copter.h"

void Copter::ctrl_pid_position_set()
{
	//OF
	if(globalFlag.of_flow_on && (globalFlag.gps_on == 0)) //只有光流时定点悬停
	{
		//normal
        g.pid_position[X].set_kP(Parameter_save.value.pid_position_save[X][KP]);
        g.pid_position[X].set_kI(0.0f);
        g.pid_position[X].set_kd_expect(0.0f);
        g.pid_position[X].set_kD(Parameter_save.value.pid_position_save[X][KD]);
        
        g.pid_position[Y].set_kP(Parameter_save.value.pid_position_save[Y][KP]);
        g.pid_position[Y].set_kI(0.0f);
        g.pid_position[Y].set_kd_expect(0.0f);
        g.pid_position[Y].set_kD(Parameter_save.value.pid_position_save[Y][KD]);
        
		//fix	
        g.pid_position_fix[X].set_kP(0.0f);
        g.pid_position_fix[X].set_kI(Parameter_save.value.pid_position_save[X][KI]);
        g.pid_position_fix[X].set_kd_expect(0.0f);
        g.pid_position_fix[X].set_kD(0.0f);
        
        g.pid_position_fix[Y].set_kP(0.0f);
        g.pid_position_fix[Y].set_kI(Parameter_save.value.pid_position_save[Y][KI]);
        g.pid_position_fix[Y].set_kd_expect(0.0f);
        g.pid_position_fix[Y].set_kD(0.0f);
	}
	//GPS
	else if(globalFlag.gps_on) //有GPS时进行导航控制
	{
        
	}
}

void Copter::position_control()
{
    float dT_s = 0.02f; //50Hz 20ms
    static float vel_fb_d_lpf[2];
    static float fb_speed_fix[2];
    float ctrl_out_position_temp[2];
    float ctrl_out_position_fix_temp[2];
    
	if(globalFlag.of_flow_on && (globalFlag.gps_on == 0)) //只有光流时定点悬停
	{
		copter.ctrl_pid_position_set();
        
		copter.ctrl_target_position[X] = copter.speed_set_h[X];
		copter.ctrl_target_position[Y] = copter.speed_set_h[Y];
		
		LPF_1_(5.0f, dT_s, imu_data.h_acc[X], vel_fb_d_lpf[X]);
		LPF_1_(5.0f, dT_s, imu_data.h_acc[Y], vel_fb_d_lpf[Y]);		

		copter.ctrl_feedback_position[X] = OF_DX2 + 0.02f * vel_fb_d_lpf[X];
		copter.ctrl_feedback_position[Y] = OF_DY2 + 0.02f * vel_fb_d_lpf[Y];
		
		fb_speed_fix[0] = OF_DX2FIX;
		fb_speed_fix[1] = OF_DY2FIX;
        
        for(u8 i =0;i<2;i++) //XY方向
		{
            copter.g.pid_position[i].set_imax(10.0f * globalFlag.taking_off); //积分幅度限幅  
            ctrl_out_position_temp[i] = 0.02f * copter.ctrl_target_position[i] +  // 前馈
                                          copter.g.pid_position[i].get_pid(dT_s,    //  周期（单位：秒） 
                                                                copter.ctrl_target_position[i], //期望值（设定值） 
                                                                copter.ctrl_feedback_position[i]); //反馈值（）
            copter.g.pid_position_fix[i].set_imax(10.0f * globalFlag.taking_off); //积分幅度限幅  
            ctrl_out_position_fix_temp[i] = 0.02f * copter.ctrl_target_position[i] +  // 前馈
                                          copter.g.pid_position_fix[i].get_pid(dT_s,    //  周期（单位：秒） 
                                                                copter.ctrl_target_position[i], //期望值（设定值） 
                                                                fb_speed_fix[i]); //反馈值（）			
			copter.ctrl_out_position[i] = ctrl_out_position_temp[i] + ctrl_out_position_fix_temp[i];	//(PD)+(I)	
		}		
    }
    else if (globalFlag.gps_on)
	{
        
    }
    else
    {
        copter.ctrl_pid_position_set();
        
        copter.ctrl_out_position[X] = MAX_CTRL_ANGLE / MAX_XY_SPEED * copter.speed_set_h[X] ;
		copter.ctrl_out_position[Y] = MAX_CTRL_ANGLE / MAX_XY_SPEED * copter.speed_set_h[Y] ;
    }
}

