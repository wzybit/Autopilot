
#include "Copter.h"
#include "filter.h"
#include "BIT_MATH.h"

_inte_fix_filter_st z_acc_fusion;
_fix_inte_filter_st z_speed_fusion, z_height_fusion;

#define N_TIMES 5

void Copter::height_data_update(void) //高度数据融合
{
    static u8 baro_offset_ok,tof_offset_ok;

	if(globalFlag.taking_off)
	{
		baro_offset_ok = 2;
	}
	else
	{
		if(baro_offset_ok == 2)
		{
			baro_offset_ok = 0;
		}
		//reset
		tof2baro_offset = 0;
	}
	
	if(baro_offset_ok >= 1)//(flag.taking_off)
	{
		ref_height_get_1 = baro_height - baro_h_offset + tof2baro_offset;//气压计相对高度，切换点跟随TOF
		//baro_offset_ok = 0;
	}
	else
	{
        baro_h_offset = baro_height;//wzy这里的偏置量是起飞前的高度
        if(globalFlag.sensor_imu_ok)
        {
            baro_offset_ok = 1;
        }
	}
	
	if((globalFlag.tof_ok || globalFlag.of_ok) && baro_offset_ok) //TOF或者OF硬件正常，且气压计记录相对值以后
	{
		if(globalFlag.tof_on || globalFlag.of_tof_on) //TOF或光流的TOF数据有效
		{
			if(globalFlag.of_tof_on) //光流带TOF，光流优先
			{
				//ref_tof_height = OF_ALT;//光流的高度 wzy
			}
			else
			{
				//ref_tof_height = tof_height_mm/10;//激光模块的高度 wzy
			}			
			
			if(tof_offset_ok == 1)
			{
				ref_height_get_2 = ref_tof_height + baro2tof_offset;//TOF参考高度，切换点跟随气压计				
				ref_height_used = ref_height_get_2;				
				tof2baro_offset += 0.5f *((ref_height_get_2 - ref_height_get_1) - tof2baro_offset);//记录气压计切换点，气压计波动大，稍微滤波一下
				//tof2baro_offset = ref_height_get_2 - ref_height_get_1;				
			}
			else
			{
				baro2tof_offset = ref_height_get_1 - ref_tof_height ; //记录TOF切换点
				tof_offset_ok = 1;
			}		
		}
		else
		{			
			tof_offset_ok = 0;			
			ref_height_used = ref_height_get_1 ;
		}
	}
	else
	{
		ref_height_used = ref_height_get_1;
	}
    uint8_t dT_ms = 10; //10ms task
	height_data_fusion(dT_ms,(s32)acc_fusion_z,(s32)(ref_height_used)); 
}

void Copter::height_data_fusion(u8 dT_ms,s32 fusion_acc_get,s32 ref_height_get)
{
	static u8 cyc_xn;    
	float hz,ntimes_hz;	    
    static s32 ref_height;
    static s32 ref_height_old;
    static s32 ref_speed;
    static s32 ref_speed_old;
    static s32 ref_acc;
    static s32 fusion_acc;
    static float wcz_acc_deadzone;	
    
	hz = safe_div(1000,dT_ms,0);
	ntimes_hz = hz/N_TIMES;
	
    ref_height = ref_height_get;
	fusion_acc = fusion_acc_get;
	wcz_acc_deadzone = LIMIT(5 *(0.996f - imu_data.z_vec[Z] *imu_data.z_vec[Z]),0,1) *10;
    
	cyc_xn ++;
	cyc_xn %= N_TIMES;
	
	if(cyc_xn == 0)
	{
		ref_speed = (ref_height - ref_height_old) *ntimes_hz;
		ref_acc = (ref_speed - ref_speed_old) *ntimes_hz;
		
		ref_height_old = ref_height;	
		ref_speed_old = ref_speed;
	}
	
	z_acc_fusion.fix_ki = 0.1f;
	z_acc_fusion.in_est = fusion_acc;
	z_acc_fusion.in_obs = ref_acc;
	z_acc_fusion.ei_limit = 200;
	inte_fix_filter(dT_ms*1e-3f,&z_acc_fusion);
	
	z_speed_fusion.fix_kp = 0.4f;
	z_speed_fusion.in_est_d = my_deadzone(z_acc_fusion.out,0,wcz_acc_deadzone);
	z_speed_fusion.in_obs = ref_speed;
	z_speed_fusion.e_limit = 200;
	fix_inte_filter(dT_ms*1e-3f,&z_speed_fusion);
	
	z_height_fusion.fix_kp = 0.4f;
	z_height_fusion.in_est_d = z_speed_fusion.out;
	z_height_fusion.in_obs = ref_height;
	//z_height_fusion.e_limit = 200;
	fix_inte_filter(dT_ms*1e-3f,&z_height_fusion);
}

void Copter::WCZ_Data_Reset()
{
	z_acc_fusion.out = 0;
	z_acc_fusion.ei = -acc_fusion_z;
	
	z_speed_fusion.out = 0;
	z_speed_fusion.e = 0;

	z_height_fusion.out = 0;
	z_height_fusion.e = 0;	
}

//高度环PID参数初始化
void Copter::ctrl_pid_height_set(void)
{
	g.pid_position[Z].set_kP(Parameter_save.value.pid_position_save[Z][KP]);
    g.pid_position[Z].set_kI(Parameter_save.value.pid_position_save[Z][KI]);
    g.pid_position[Z].set_kd_expect(0.0f);
    g.pid_position[Z].set_kD(Parameter_save.value.pid_position_save[Z][KD]);
}

//高度方向油门速率环PID参数初始化
void Copter::ctrl_pid_height_speed_set(void)
{
	g.pid_height_speed.set_kP(Parameter_save.value.pid_height_speed_save[KP]);
    g.pid_height_speed.set_kI(Parameter_save.value.pid_height_speed_save[KI]);
    g.pid_height_speed.set_kd_expect(0.0f);
    g.pid_height_speed.set_kD(0.0f);
}


void Copter::height_control()
{
    float dT_s = 10e-3f; //10ms
    static float ctrl_height_speed_set;
    static float w_acc_z_lpf;
    static float err_i_comp;
    static float ctrl_out_height_speed_temp;
    
	copter.auto_take_off_land_task();//自动起飞降落任务，一键起飞和降落还没加，以后自动驾驶应该要加上    wzy
	
	ctrl_height_speed_set = copter.speed_set_h[Z] + copter.auto_taking_off_speed;//摇杆加自动起飞速度，起飞后后者为0
	
	copter.ctrl_target_position[Z] += ctrl_height_speed_set * dT_s;
	copter.ctrl_target_position[Z]  = LIMIT(copter.ctrl_target_position[Z], 
                                           copter.ctrl_feedback_position[Z] - 200.0f, 
                                           copter.ctrl_feedback_position[Z] + 200.0f);
	copter.ctrl_feedback_position[Z] = z_height_fusion.out;//wzy 这里都用float类型 应该可以吧
	
	if(globalFlag.taking_off == 1)
	{
        copter.g.pid_position[Z].set_imax(0.0f); //积分幅度限幅  
        copter.ctrl_out_position[Z] = copter.g.pid_position[Z].get_pid(dT_s,    //  周期（单位：秒） 
                                                                copter.ctrl_target_position[Z], //期望值（设定值） 
                                                                copter.ctrl_feedback_position[Z]); //反馈值（） 
	}
	else
	{
		copter.ctrl_target_position[Z] = copter.ctrl_feedback_position[Z];
		copter.ctrl_out_position[Z] = 0;		
	}
	
	copter.ctrl_out_position[Z]  = LIMIT(copter.ctrl_out_position[Z], -150.0f, 150.0f);
    	
	copter.ctrl_target_height_speed = 0.6f * ctrl_height_speed_set + copter.ctrl_out_position[Z];//速度前馈0.6f，直接给速度
	
	w_acc_z_lpf += 0.1f *(imu_data.w_acc[Z] - w_acc_z_lpf); //低通滤波

    //微分先行，下边PID函数微分系数为0 wzy注意
	copter.ctrl_feedback_height_speed = z_speed_fusion.out + Parameter_save.value.pid_height_speed_save[KD] * w_acc_z_lpf;
		
    copter.g.pid_height_speed.set_imax((THR_INTE_LIM * 10.0f - err_i_comp) * globalFlag.taking_off); //积分幅度限幅  
    ctrl_out_height_speed_temp = copter.g.pid_height_speed.get_pid(dT_s,    //  周期（单位：秒） 
                                                                copter.ctrl_target_height_speed, //期望值（设定值） 
                                                                copter.ctrl_feedback_height_speed); //反馈值（） 
	if(globalFlag.taking_off == 1)
	{
		LPF_1_(1.0f, dT_s, THR_START *10.0f, err_i_comp);//err_i_comp = THR_START *10;			
	}
	else
	{
		err_i_comp = 0;
	}	
	
	copter.ctrl_out_height_speed = globalFlag.taking_off * (ctrl_out_height_speed_temp + err_i_comp);//wzy
	copter.ctrl_out_height_speed = LIMIT(copter.ctrl_out_height_speed, 0, MAX_THR_SET *10.0f);	
	
	copter.motor_input_throttle = copter.ctrl_out_height_speed; //wzy由此可知油门控制为速率模式
}

void Copter::auto_take_off_land_task()
{
    u8 dT_ms = 10; // 100Hz
	static u16 take_off_ok_cnt;	
	
	if(globalFlag.unlockStatus)
	{
		if(globalFlag.taking_off)
		{	
			if(globalFlag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
			{
				globalFlag.auto_take_off_land = AUTO_TAKE_OFF;		
			}
		}
	}
	else
	{
		auto_taking_off_speed = 0.0f;	
		globalFlag.auto_take_off_land = AUTO_TAKE_OFF_NULL;	
	}
	
	if(globalFlag.auto_take_off_land ==AUTO_TAKE_OFF) //准备自动起飞
	{
		//设置最大起飞速度
		float max_take_off_vel = LIMIT(Parameter_save.value.auto_take_off_speed, 20.0f, 200.0f);
		//
		take_off_ok_cnt += dT_ms;
		auto_taking_off_speed = 2.0f * (Parameter_save.value.auto_take_off_height - z_height_fusion.out);
		//计算起飞速度
		auto_taking_off_speed = LIMIT(auto_taking_off_speed, 0.0f, max_take_off_vel);
		
		//退出起飞流程条件1，满足高度或者流程时间大于5000毫秒。
		if(take_off_ok_cnt >= 5000 || (Parameter_save.value.auto_take_off_height - ctrl_feedback_position[Z] < 2.0f))//(auto_ref_height>AUTO_TAKE_OFF_HEIGHT)
		{
			globalFlag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;			
		}
		//退出起飞流程条件2，2000毫秒后判断用户正在控制油门。
		if(take_off_ok_cnt > 2000 && ABS(speed_set_h_norm[Z]) > 0.1f)// 一定已经taking_off,如果还在推杆，退出起飞流程
		{
			globalFlag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
		}	
	}
	else 
	{
		take_off_ok_cnt = 0;		
		if(globalFlag.auto_take_off_land ==AUTO_TAKE_OFF_FINISH)
		{
			auto_taking_off_speed = 0.0f;			
		}		
	}
	
	if(globalFlag.auto_take_off_land == AUTO_LAND) //准备自动降落 wzy 这个降落还很不完善 需要考虑离地高度
	{
		//设置自动下降速度
		auto_taking_off_speed = -LIMIT(Parameter_save.value.auto_landing_speed, 20.0f, 200.0f);
	}
}
