/*
 *@File         : radio.cpp
 *@Author       : wangbo
 *@Date         : Aug 1, 2017
 *@Copyright    : 2018 Beijing Institute of Technology. All right reserved.
 *@Warning      : This content is limited to internal use within the Complex Industrial Control Laboratory of Beijing Institute of Technology.
                  Prohibition of leakage and other commercial purposes.
 *@description  : Remote control channel data reading and processing               
 */

#include "Copter.h"

void Copter::Remote_Control_Init(void)
{
	if(globalConfig.rcInMode == SBUS)
	{
		USART2_Init(100000); //SBUS协议数据最终采用串口中断模式读取
	}
	else if(globalConfig.rcInMode == PWM)
	{
		PWM_In_Init(globalConfig.rcInMode);//这里接收机数据读取又分为PPM和PWM两种方式
	}
}

void Copter::rc_in_para_init()
{    
    for(u8 i = 0; i < CH_NUM; i++)
	{
        RC_CH[i] = 0;
    }
	/*
	 * radio的范围是1000~2000，radio_min = 1000
	 * 这个在angle_to_pwm,pwm_to_angle等中都有用
	 */
	g.channel_roll.radio_min     = 1000;
	g.channel_pitch.radio_min    = 1000;
	g.channel_throttle.radio_min = 1000;
	g.channel_rudder.radio_min   = 1000;
	g.channel_5.radio_min             = 1000;
	g.channel_6.radio_min             = 1000;
	g.channel_7.radio_min             = 1000;
	g.channel_8.radio_min             = 1000;

	g.channel_roll.radio_max     = 2000;
	g.channel_pitch.radio_max    = 2000;
	g.channel_throttle.radio_max = 2000;
	g.channel_rudder.radio_max   = 2000;
	g.channel_5.radio_max             = 2000;
	g.channel_6.radio_max             = 2000;
	g.channel_7.radio_max             = 2000;
	g.channel_8.radio_max             = 2000;

    /// trim 本身是切除的意思，radio_trim也就指的是中间值，因为futaba等其他遥控器的输入我们都映射为1000~2000
    /// 但是控制上需要的是正负控制，有正有负，中间值是1500，当遥控器输入小于1500时和大于1500时，是不同的偏转方向
	g.channel_roll.radio_trim     = 1500;
	g.channel_pitch.radio_trim    = 1500;
    g.channel_throttle.radio_trim = 0; /// 3 is not trimed  这里arducopter中注释说，throttle是没有trim的，但是没有的话，我的运行就有错误，到底是有没有呢,初始化的时候油门通道是不一样的，用的是set_range
	g.channel_rudder.radio_trim   = 1500;
    
	g.channel_5.radio_trim             = 1500;/// 下面这些一般都是舵机，都是有trim中立位的，只有油门是没有中立值的
	g.channel_6.radio_trim             = 1500;
	g.channel_7.radio_trim             = 1500;
	g.channel_8.radio_trim             = 1500;

	g.channel_roll.set_reverse(0); /// 不取反
	g.channel_pitch.set_reverse(0);
	g.channel_rudder.set_reverse(0);
	g.channel_throttle.set_reverse(0);
	g.channel_5.set_reverse(0);

	/// set rc dead zones
    /// dead zone指的是死区的意思，也就是当遥控器的控制输入在-dead_zone ~ +dead_zone之间的时候，依然认为是0
    /// 相当于做了死区特性
	g.channel_roll.dead_zone 	 = 60;
	g.channel_pitch.dead_zone 	 = 60;
    g.channel_throttle.dead_zone = 60;
	g.channel_rudder.dead_zone 	 = 60;


	/// set rc channel ranges
    /// 因为channel是所有通道的类，但是角度控制和油门控制并不一样，油门从0开始，只有正没有负，角度比如滚转俯仰偏航都是由正负的，所以范围是[-4500~+4500]
	g.channel_roll.set_angle(4500); /// set_angle做了两件事，一是把_high变量赋值，一是设置type是角度[-4500~+4500]还是范围[0~+1000]
	g.channel_pitch.set_angle(4500);
	g.channel_throttle.set_range(0, 1000); /// 注意这里是set_range;！！！！！！误删除
	g.channel_rudder.set_angle(4500);
    
    //set auxiliary ranges
    g.channel_5.set_range(0,1000);
    g.channel_6.set_range(0,1000);
    g.channel_7.set_range(0,1000);
    g.channel_8.set_range(0,1000);
}

void Copter::read_radio()
{
    if(globalFlag.init_ok)	
	{
		/////////////获得通道数据////////////////////////
		if(globalConfig.rcInMode == PWM)
		{
			for(u8 i=0;i<CH_NUM;i++)
			{
				copter.RC_CH[i] = 1.25f *((s16)Rc_Pwm_In[i] - 1500); //1100 -- 1900us,处理成大约+-500摇杆量
				copter.RC_CH[i] = LIMIT(copter.RC_CH[i],-500,500);//限制到+—500
			}
		}
		else if(globalConfig.rcInMode == PPM)
		{
			for(u8 i=0;i<CH_NUM;i++)
			{
                copter.RC_CH[i] = 1.25f *((s16)Rc_Ppm_In[i] - 1100); //700 -- 1500us,处理成大约+-500摇杆量
				copter.RC_CH[i] = LIMIT(copter.RC_CH[i],-500,500);//限制到+—500
			}		
		}
		else//sbus
		{
			for(u8 i=0;i<CH_NUM;i++)
			{
				copter.RC_CH[i] = 0.65f *((s16)rc_sbus_in[i] - 1024); //248 --1024 --1800,处理成大约+-500摇杆量
				copter.RC_CH[i] = LIMIT(copter.RC_CH[i],-500,500);//限制到+—500
                
                //wzy
//                copter.RC_CH[i] = (int16_t)((rc_sbus_in[i] - SBUS_IN_MIN) 
//                * 1000.0f / (SBUS_IN_MAX - SBUS_IN_MIN) - 500.0f); //转换为-500~500
//                copter.RC_CH[i] = LIMIT(copter.RC_CH[i], -500, 500); //限制到+—500
			}					
		}         		
		//stick_function(dT_ms); //摇杆触发功能监测//wzy暂时不用
		
		//fail_safe_check(dT_ms); //失控保护检查 //wzy暂时不用
	}
    
    /*
     * set_pwm做两件事，
     * 1.是给radion_in赋值，作为从rc_channel读取回来的数
     * 2.是把control_in = pwm_to_angle(radio)，也就是把读取回来的pwm转为-4500～+4500角度控制值
     */    
//  copter.g.channel_roll.set_pwm(copter.RC_CH[0]);
//	copter.g.channel_pitch.set_pwm(copter.RC_CH[1]);
//	copter.g.channel_throttle.set_pwm(copter.RC_CH[2]);
//	copter.g.channel_rudder.set_pwm(copter.RC_CH[3]);
//    
//    copter.g.channel_5.set_pwm(copter.RC_CH[4]);
//    copter.g.channel_6.set_pwm(copter.RC_CH[5]);
//    copter.g.channel_7.set_pwm(copter.RC_CH[6]);
//    copter.g.channel_8.set_pwm(copter.RC_CH[7]);
}

void Copter::unlock_check(u8 dT_ms)
{
    //解锁
    if(globalFlag.unlockStatus == 0) //当前状态为上锁状态时
	{
        if(globalFlag.unlock_cmd) //有解锁命令时
        {		
            if( globalFlag.power_ok ==1 && para_sta.save_trig == 0)//只有电池电压非最低并且没有操作flash时，才允许进行解锁
            {
                if(globalFlag.acc_ok && globalFlag.gyro_ok)
                {
                    if(globalFlag.baro_ok)
                    {
                        if(globalFlag.sensor_imu_ok  ) //imu正常时，才允许解锁
                        {                            
                                globalFlag.unlockStatus = globalFlag.unlock_cmd; //允许解锁
                                GCS_SendString("Unlock OK!");                        
                        }
                        else
                        {
                            globalFlag.unlock_cmd = 0; //重置解锁命令
                            GCS_SendString("Unlock Fail!"); //imu异常，不允许解锁
                        }
                    }
                    else
                    {
                        globalFlag.unlock_cmd = 0; //重置解锁命令
                        LED_STA.errBaro = 1; //气压计异常，不允许解锁。
                        GCS_SendString("Unlock Fail!");
                    }
                }
                else
                {
                    globalFlag.unlock_cmd = 0; //重置解锁命令
                    LED_STA.errMpu = 1; //惯性传感器异常，不允许解锁。
                    GCS_SendString("Unlock Fail!");
                }
            }
            else
            {
                globalFlag.unlock_cmd = 0; //重置解锁命令
                GCS_SendString("Power Low,Unlock Fail!"); //电池电压异常，不允许解锁
            }
        }
	}
	else //当前状态为解锁状态时 
	{
		if(globalFlag.unlock_cmd == 0)
		{
			GCS_SendString(" FC Output Locked! ");
		}		
		globalFlag.unlockStatus = globalFlag.unlock_cmd; //允许上锁
	}
	
	//所有功能判断，都要油门在低值时才进行
	if(RC_CH[CH_THR] < -300  )
	{
		//飞控上锁、解锁检测
		if(RC_CH[CH_PIT] < -300 && RC_CH[CH_ROL] > 300 && RC_CH[CH_YAW] < -300)
		{
			globalFlag.locking = 1;
		}
		else if(RC_CH[CH_PIT] < -300 && RC_CH[CH_ROL] < -300 && RC_CH[CH_YAW] > 300)
		{
			globalFlag.locking = 1;
		}
		else
		{
			globalFlag.locking = 0;
		}
			
		static uint16_t locking_cnt;
        if(globalFlag.locking)//如果满足摇杆条件，则进行时间积分
        {
            if(locking_cnt!=0)
            {
                locking_cnt += dT_ms;
            }
        }
        else//不满足条件，积分恢复1
        {
            locking_cnt = 1;
        }
        //时间积分满足时间阈值，则触发标记
        if(locking_cnt >= 1000) //1s  
        {
            if(globalFlag.unlockStatus) //当前是解锁状态
            {
                globalFlag.unlock_cmd = 0; //触发功能标记 上锁命令
            }
            else //当前是上锁状态
            {
                globalFlag.unlock_cmd = 1; //触发功能标记 解锁命令
            }
            locking_cnt = 0;
        }    
	}
	else //油门高
	{
		globalFlag.locking = 0; 
	}
}

