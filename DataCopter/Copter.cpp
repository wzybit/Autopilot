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
    all_pid_init();    //����������PID������ʼ��
    
    globalFlag.power_ok = 1; //��Դ��⿪�� ����ֱ����1 ��Ҫɾ��
    globalFlag.mems_temperature_ok = 1; //���Դ��������ȿ��� ����ֱ����1 ��Ҫɾ��
    sensor.acc_z_auto_CALIBRATE = 1; //�����Զ���׼Z��
    sensor.acc_CALIBRATE = 1; //�����Զ�У׼���ٶȼ�
	sensor.gyr_CALIBRATE = 2; //�����Զ�У׼������
    
    return (1);
}

void Copter::driver_init(void)
{
    NVIC_PriorityGroupConfig(NVIC_GROUP);	//�ж����ȼ��������
    
    SysTick_Config(SystemCoreClock / 1000); // system tick : 1000 ~ 1ms     �δ�ʱ������
    delay_ms(100);

    Driver_LED_Init();    
    
    Flash_w25qxx_Init();      //����FLASHоƬ������ʼ��
	Parameter_Read();         //�������ݳ�ʼ��
		
    globalConfig.rcInMode = SBUS;
	Remote_Control_Init();    //��ʼ��ң�������ջ��ɼ�����
	
	PWM_Out_Init(400);	      //��ʼ������������ ���400Hz PWM	
	delay_ms(100);			  //��ʱ
	
    I2C_Soft_Init(I2C_1);     //I2C_1 ��ȡMS5611��ѹ������
    I2C_Soft_Init(I2C_2);     //I2C_2 ��ȡMPU6050�����Ǽ��ٶȼ����ݺ�AK8975����������
    I2C_Soft_Init(I2C_3);     //I2C_3 ����I2C ������չ����
    
    //MPU6050�����Ǽ��ٶȼƳ�ʼ��������ʼ���ɹ����������Ǻͼ��ٶȵĳ�ʼ���ɹ���־λ��ֵ ��ʼ������ڳ�ʼ��������
    MPU6050_Init(20);     
    Center_Pos_Set();                   //�����������IMU���Դ�������ƫ����
    globalFlag.mag_ok = AK8975_Init();  //���̳�ʼ�� �������OK
    MS5611_Init();                      //MS5611��ѹ�Ƴ�ʼ�� ��ʼ������ڳ�ʼ��������

	//Usb_Hid_Init();		  //�ɿ�usb�ӿڵ�hid��ʼ�� wzy �����޸Ĳ���һ��
	delay_ms(100);			  //��ʱ
	
//	USART1_Init(115200);	  //��ʱû���õ� ������115200
	delay_ms(10);			  //��ʱ	
	USART3_Init(115200);      //���� �����վͨ���Լ���ӡ ������115200
	delay_ms(10);			  //��ʱ
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
		//��ȡ�����Ǽ��ٶȼ�����
		MPU6050_Read();
        
		cnt ++;
		cnt %= 20;
		if(cnt == 0)
		{
			//��ȡ�������̴���������
			AK8975_Read();
			//��ȡ��ѹ������	
            baro_height = MS5611_Read();
		}
	}	
    
    IMU_Data_Prepare(1); //���Դ���������׼��
    
    if(globalFlag.unlockStatus)
    {
        imu_data.gravity_reset = imu_data.mag_reset = 0;
        reset_imu_flag = 0;
    }
    else if(reset_imu_flag == 0 )//&& flag.motionless == 1)
    {
        imu_data.gravity_reset = 1;//�Զ���λ	
        sensor.gyr_CALIBRATE = 2;//У׼�����ǣ�������
        reset_imu_flag = 1;     //�Ѿ���λ��λ���
    }
    
	/*У׼�У���λ��������*/
	if(sensor.gyr_CALIBRATE != 0 || sensor.acc_CALIBRATE != 0 ||sensor.acc_z_auto_CALIBRATE)
	{
		imu_data.gravity_reset = 1;
	}
	
	/*��λ��������ʱ����Ϊ������ʧЧ*/
	if(imu_data.gravity_reset == 1)
	{
		globalFlag.sensor_imu_ok = 0;
		LED_STA.rst_imu = 1;
		WCZ_Data_Reset(); //��λ�߶������ں�
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
    
    // ��̬���㣬���£��ں� ע��������̬���� ����������Ϊ���� ���ٶ������ģΪ+-8G
    uint8_t dT_ms = 1; //1ms task
    float mag_val_f[VEC_XYZ];
    for(u8 i = 0;i<3;i++)
    {
        mag_val_f[i] = (float)mag.val[i];
    }
    IMU_update(dT_ms * 1e-3f, sensor.Gyro_rad, sensor.Acc_cmss, mag_val_f, &imu_data);
    
    //��ȡ�ںϼ��ٶ����� Ϊ֮��ĸ߶Ⱥ�λ�ÿ��� �ṩ���õļ��ٶ�����
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
	
	if(flight_mode_old != globalConfig.flightMode) //ҡ�˶�Ӧģʽ״̬�ı�
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
    
	speed_set_h_norm[Z] = my_deadzone(RC_CH[CH_THR], 0.0f, 50.0f) *0.0023f; //����-50 ~ 50����
	speed_set_h_norm_lpf[Z] += 0.5f *(speed_set_h_norm[Z] - speed_set_h_norm_lpf[Z]);
		
	if(globalFlag.unlockStatus) 
	{	
		if(speed_set_h_norm[Z]>0.01f && globalFlag.motor_preparation == 1) //������ʱ ���ҵ��������
        {
			globalFlag.taking_off = 1; //���
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
			globalFlag.flying = 1;  //��ɺ�1�룬��Ϊ�Ѿ��ڷ���
		}
		
		if(speed_set_h_norm[Z]>0)
		{
			/*���������ٶ�*/
			vel_z_tmp[0] = (speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_UP);
		}
		else
		{
			/*�����½��ٶ�*/
			vel_z_tmp[0] = (speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_DW);
		}

		//vel_z_tmp[1] = vel_z_tmp[0] + program_ctrl.vel_cmps_h[Z]; //wzy
		vel_z_tmp[1] = vel_z_tmp[0];
		vel_z_tmp[1] = LIMIT(vel_z_tmp[1], -MAX_Z_SPEED_DW, MAX_Z_SPEED_UP);
		speed_set_h[Z] += LIMIT((vel_z_tmp[1] - speed_set_h[Z]), -0.8f, 0.8f);//������������
	}
	else
	{
		speed_set_h[Z] = 0 ;
	}
	float speed_set_tmp[2], max_speed_lim;
	//XY�����ٶ��趨��
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
	
	//���Ź�һֵС��0.1���� ����ҡ��ֵС��-350���������Զ�����
	if( speed_set_h_norm[Z] < 0.1f && (RC_CH[CH_THR] < -350 || globalFlag.auto_take_off_land == AUTO_LAND))
	{
		/*�������������С��250����û�����ֶ��������������У�����1�룬��Ϊ��½��Ȼ������*/
		if(motor_input_throttle < 250 && globalFlag.unlockStatus == 1 && globalFlag.locking == 0)//ABS(wz_spe_f1.out <20 ) //��Ӧ�� �����ٶ��������ٶ�С����20����ÿ�롣
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
				globalFlag.unlock_cmd =0;	//��������
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
	
	/*����״̬��λ*/
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
	if(globalFlag.of_ok)//����ģ��
	{
		if(OF_QUALITY > 20) //������������20 /*�����ڷ���֮ǰ*/����Ϊ�������ã��ж������ӳ�ʱ��Ϊ1��
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
		
		if(OF_ALT < 500 && globalConfig.flightMode == LOC_HOLD)//�����߶�500cm����Ч
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
	
	if(globalFlag.tof_ok)//����ģ��
	{
//		if(tof_height_mm<1900)  wzy ��ʱ������Ӽ���ģ��
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
