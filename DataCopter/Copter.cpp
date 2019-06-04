 /*
 *@File         : Copter.cpp
 *@Author       : wangbo
 *@Date         : Nov 4, 2017
 *@Copyright    : 2018 Beijing Institute of Technology. All right reserved.
 *@Warning      : This content is limited to internal use within the Complex Industrial Control Laboratory of Beijing Institute of Technology.
                  Prohibition of leakage and other commercial purposes.
 *@description  : Copter class implementation functions called by the DataCopter and other source files            
 */

#include "Copter.h"

Copter copter;
struct T_GLOBAL_TIME globalSystemTime;
struct T_GLOBAL_CONFIG globalConfig;
struct T_GLOBAL_FLAG globalFlag;
struct T_GLOBAL_SAVE_DATA globalSaveData;

Copter::Copter()
{
//    radius_of_earth 	= 6378100;	// meters  wzy
//    gravity 			= 9.81;		// meters/ sec^2
}

uint8_t Copter::setup( void )
{     
    driver_init(); /// hardware init, this must be the first
    
    DEBUG_PRINTF("Welcome to BitPilot \n"); // after usart init
        
    rc_in_para_init(); /// sets up rc channels from radio
    all_pid_init();    //控制器所有PID参数初始化
    
    globalFlag.power_ok = 1; //电源检测开关 这里直接置1 不要删除
    globalFlag.mems_temperature_ok = 1; //惯性传感器加热开关 这里直接置1 不要删除
    sensor.acc_z_auto_CALIBRATE = 1; //开机自动对准Z轴
    sensor.acc_CALIBRATE = 1; //开机自动校准加速度计
	sensor.gyr_CALIBRATE = 2; //开机自动校准陀螺仪
    
    return (1);
}

void Copter::driver_init(void)
{
    NVIC_PriorityGroupConfig(NVIC_GROUP);	//中断优先级组别设置
    
    SysTick_Config(SystemCoreClock / 1000); // system tick : 1000 ~ 1ms     滴答时钟设置
    delay_ms(100);

    Driver_LED_Init();    
    
    Flash_w25qxx_Init();      //板载FLASH芯片驱动初始化
	Parameter_Read();         //参数数据初始化
		
    globalConfig.rcInMode = SBUS;
	Remote_Control_Init();    //初始化遥控器接收机采集功能
	
	PWM_Out_Init(400);	      //初始化电调输出功能 输出400Hz PWM	
	delay_ms(100);			  //延时
	
    I2C_Soft_Init(I2C_1);     //I2C_1 读取MS5611气压计数据
    I2C_Soft_Init(I2C_2);     //I2C_2 读取MPU6050陀螺仪加速度计数据和AK8975磁力计数据
    I2C_Soft_Init(I2C_3);     //I2C_3 外置I2C 用于扩展功能
    
    //MPU6050陀螺仪加速度计初始化，若初始化成功，则将陀螺仪和加速度的初始化成功标志位赋值 初始化标记在初始化函数里
    MPU6050_Init(20);     
    Center_Pos_Set();                   //设置重心相对IMU惯性传感器的偏移量
    globalFlag.mag_ok = AK8975_Init();  //罗盘初始化 标记罗盘OK
    MS5611_Init();                      //MS5611气压计初始化 初始化标记在初始化函数里

	//Usb_Hid_Init();		  //飞控usb接口的hid初始化 wzy 可以修改测试一下
	delay_ms(100);			  //延时
	
//	USART1_Init(115200);	  //暂时没有用到 波特率115200
	delay_ms(10);			  //延时	
	USART3_Init(115200);      //数传 与地面站通信以及打印 波特率115200
	delay_ms(10);			  //延时
}

void Copter::all_pid_init(void)
{
    ctrl_pid_rate_set();
    ctrl_pid_angle_set(); 
    ctrl_pid_height_set();   
    ctrl_pid_height_speed_set();     
    ctrl_pid_position_set();
}

void Copter::update_onboard_sensor_input( void )
{
    static u8 cnt;
    static u8 reset_imu_flag;
	if(globalFlag.init_ok)
	{
		//读取陀螺仪加速度计数据
		MPU6050_Read();
        
		cnt ++;
		cnt %= 20;
		if(cnt == 0)
		{
			//读取电子罗盘磁力计数据
			AK8975_Read();
			//读取气压计数据	
            baro_height = MS5611_Read();
		}
	}	
    
    IMU_Data_Prepare(1); //惯性传感器数据准备
    
    if(globalFlag.unlockStatus)
    {
        imu_data.gravity_reset = imu_data.mag_reset = 0;
        reset_imu_flag = 0;
    }
    else if(reset_imu_flag == 0 )//&& flag.motionless == 1)
    {
        imu_data.gravity_reset = 1;//自动复位	
        sensor.gyr_CALIBRATE = 2;//校准陀螺仪，不保存
        reset_imu_flag = 1;     //已经置位复位标记
    }
    
	/*校准中，复位重力方向*/
	if(sensor.gyr_CALIBRATE != 0 || sensor.acc_CALIBRATE != 0 ||sensor.acc_z_auto_CALIBRATE)
	{
		imu_data.gravity_reset = 1;
	}
	
	/*复位重力方向时，认为传感器失效*/
	if(imu_data.gravity_reset == 1)
	{
		globalFlag.sensor_imu_ok = 0;
		LED_STA.rst_imu = 1;
		WCZ_Data_Reset(); //复位高度数据融合
	}
	else if(imu_data.gravity_reset == 0)
	{	
		if(globalFlag.sensor_imu_ok == 0)
		{
			globalFlag.sensor_imu_ok = 1;
			LED_STA.rst_imu = 0;
			GCS_SendString("IMU OK!");
		}
	}
    
    // 姿态计算，更新，融合 注意这里姿态解算 陀螺仪数据为弧度 加速度三轴的模为+-8G
    uint8_t dT_ms = 1; //1ms task
    float mag_val_f[VEC_XYZ];
    for(u8 i = 0;i<3;i++)
    {
        mag_val_f[i] = (float)mag.val[i];
    }
    IMU_update(dT_ms * 1e-3f, sensor.Gyro_rad, sensor.Acc_cmss, mag_val_f, &imu_data);
    
    //获取融合加速度数据 为之后的高度和位置控制 提供可用的加速度数据
    acc_fusion_z += 0.1f * (imu_data.w_acc[Z] - acc_fusion_z);
    
    acc_fusion_x += 0.02f * (imu_data.w_acc[X] - acc_fusion_x);
	acc_fusion_y += 0.02f * (imu_data.w_acc[Y] - acc_fusion_y);
}

void Copter::update_MAG(void)
{       
    uint8_t dT_ms = 20; //20ms task
    
    Mag_Get();
	Mag_Data_Deal_Task(dT_ms, mag_val, imu_data.z_vec[Z], sensor.Gyro_deg[X], sensor.Gyro_deg[Z]);
}

void Copter::update_height(void)
{
    copter.height_data_update();
}
void Copter::update_GPS()
{
}

void Copter::update_flight_mode(void)
{  
    static uint8_t flight_mode_old = 0;

    if(copter.RC_CH[AUX1]<-300)
    {
        globalConfig.flightMode = STABILIZE;
    }
    else if(copter.RC_CH[AUX1]<200)
    {
        globalConfig.flightMode = LOC_HOLD;
    }
    else
    {
        globalConfig.flightMode = RETURN_HOME;
    }
	
	if(flight_mode_old != globalConfig.flightMode) //摇杆对应模式状态改变
	{
		flight_mode_old = globalConfig.flightMode;
		
		globalFlag.rc_loss_back_home = 0;
	}
}
void Copter::update_flight_status(void)
{
    uint8_t dT_ms = 1; //1ms task
    static int16_t landing_cnt;
    static int16_t flying_cnt;
	static float vel_z_tmp[2];
    
	speed_set_h_norm[Z] = my_deadzone(RC_CH[CH_THR], 0.0f, 50.0f) *0.0023f; //设置-50 ~ 50死区
	speed_set_h_norm_lpf[Z] += 0.5f *(speed_set_h_norm[Z] - speed_set_h_norm_lpf[Z]);
		
	if(globalFlag.unlockStatus) 
	{	
		if(speed_set_h_norm[Z]>0.01f && globalFlag.motor_preparation == 1) //推油门时 并且电机启动后
        {
			globalFlag.taking_off = 1; //起飞
		}	
	}		
	
	if(globalFlag.taking_off)
	{			
		if(flying_cnt<1000)//800ms
		{
			flying_cnt += dT_ms;
		}
		else
		{			
			globalFlag.flying = 1;  //起飞后1秒，认为已经在飞行
		}
		
		if(speed_set_h_norm[Z]>0)
		{
			/*设置上升速度*/
			vel_z_tmp[0] = (speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_UP);
		}
		else
		{
			/*设置下降速度*/
			vel_z_tmp[0] = (speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_DW);
		}

		//vel_z_tmp[1] = vel_z_tmp[0] + program_ctrl.vel_cmps_h[Z]; //wzy
		vel_z_tmp[1] = vel_z_tmp[0];
		vel_z_tmp[1] = LIMIT(vel_z_tmp[1], -MAX_Z_SPEED_DW, MAX_Z_SPEED_UP);
		speed_set_h[Z] += LIMIT((vel_z_tmp[1] - speed_set_h[Z]), -0.8f, 0.8f);//限制增量幅度
	}
	else
	{
		speed_set_h[Z] = 0 ;
	}
	float speed_set_tmp[2], max_speed_lim;
	//XY方向速度设定量
	speed_set_h_norm[X] = my_deadzone(+RC_CH[CH_PIT], 0, 50) * 0.0022f;
	speed_set_h_norm[Y] = my_deadzone(-RC_CH[CH_ROL], 0, 50) * 0.0022f;
		
	LPF_1_(3.0f,dT_ms*1e-3f, speed_set_h_norm[X], speed_set_h_norm_lpf[X]);
	LPF_1_(3.0f,dT_ms*1e-3f, speed_set_h_norm[Y], speed_set_h_norm_lpf[Y]);
	
	max_speed_lim = MAX_XY_SPEED;
	
	if(globalFlag.of_flow_on)
	{
		//max_speed_lim = 1.5f *wcz_hei_fus.out; //wzy
		max_speed_lim = LIMIT(max_speed_lim,50,150);
	}	
		
//	speed_set_tmp[X] = max_speed_lim * speed_set_h_norm_lpf[X] + program_ctrl.vel_cmps_h[X]; //wzy
//	speed_set_tmp[Y] = max_speed_lim * speed_set_h_norm_lpf[Y] + program_ctrl.vel_cmps_h[Y]; //wzy
    
    speed_set_tmp[X] = max_speed_lim * speed_set_h_norm_lpf[X]; 
	speed_set_tmp[Y] = max_speed_lim * speed_set_h_norm_lpf[Y];
	
	length_limit(&speed_set_tmp[X],&speed_set_tmp[Y], max_speed_lim, speed_set_h);
	
	//油门归一值小于0.1并且 油门摇杆值小于-350或者启动自动降落
	if( speed_set_h_norm[Z] < 0.1f && (RC_CH[CH_THR] < -350 || globalFlag.auto_take_off_land == AUTO_LAND))
	{
		/*油门最终输出量小于250并且没有在手动解锁上锁过程中，持续1秒，认为着陆，然后上锁*/
		if(motor_input_throttle < 250 && globalFlag.unlockStatus == 1 && globalFlag.locking == 0)//ABS(wz_spe_f1.out <20 ) //还应当 与上速度条件，速度小于正20厘米每秒。
		{
			if(landing_cnt<1500)
			{
				landing_cnt += dT_ms;
			}
			else
			{
				landing_cnt =0;	
				flying_cnt = 0;
				globalFlag.taking_off = 0;			
				globalFlag.flying = 0;
				globalFlag.unlock_cmd =0;	//上锁命令
			}
		}
		else
		{
			landing_cnt = 0;
		}
	}
	else
	{
		landing_cnt  = 0;
	}
	
	/*飞行状态复位*/
	if(globalFlag.unlockStatus == 0)
	{		
		landing_cnt = 0; 
        flying_cnt = 0;        
		globalFlag.taking_off = 0;
		globalFlag.flying = 0;
		globalFlag.rc_loss_back_home = 0;
	}
}

void Copter::update_external_sensor_switch(void)
{
    uint8_t dT_ms = 1; //1ms task
    static uint8_t of_light_ok;
    static uint16_t of_light_delay;
	if(globalFlag.of_ok)//光流模块
	{
		if(OF_QUALITY > 20) //光流质量大于20 /*或者在飞行之前*/，认为光流可用，判定可用延迟时间为1秒
		{
			if(of_light_delay<1000)
			{
				of_light_delay += dT_ms;
			}
			else
			{
				of_light_ok = 1;
			}
		}
		else
		{
			of_light_delay =0;
			of_light_ok = 0;
		}
		
		if(OF_ALT < 500 && globalConfig.flightMode == LOC_HOLD)//光流高度500cm内有效
		{		
			if(of_light_ok)
			{
				globalFlag.of_flow_on = 1;
			}
			else
			{
				globalFlag.of_flow_on = 0;
			}
			globalFlag.of_tof_on = 1;
		}
		else
		{
			globalFlag.of_tof_on = 0;
			globalFlag.of_flow_on = 0;
		}			
	}
	else
	{
		globalFlag.of_flow_on = globalFlag.of_tof_on = 0;
	}
	
	if(globalFlag.tof_ok)//激光模块
	{
//		if(tof_height_mm<1900)  wzy 暂时不用外接激光模块
//		{
//			globalFlag.tof_on = 1;
//		}
//		else
//		{
//			globalFlag.tof_on = 0;
//		}
	}
	else
	{
		globalFlag.tof_on = 0;
	}
}

void Copter::arm_motors_check() 
{
    uint8_t dT_ms = 10; //10ms task
    if(globalFlag.init_ok)	
	{
        copter.unlock_check(dT_ms);
    }
}  

void Copter::wait_ms(uint32_t ms)
{
    delay_ms(ms);
}

void Copter::wait_us(uint32_t us)
{
    delay_us(us);
}
