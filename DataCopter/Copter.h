/*
 * Copter.h
 *
 *  Created on: 2018-3-12
 *      Author: wangbo
 */
#ifndef COPTER_H_
#define COPTER_H_

/*
 * 系统相关头文件
 */
#include "stm32f4xx.h"

//C标准头文件
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

/*
 * 硬件驱动相关头文件
 */
#include "driver_led.h"
#include "driver_i2c.h"
//#include "driver_spi.h"
#include "driver_pwm_in.h"
#include "driver_pwm_out.h"
#include "driver_usart.h"
#include "flash_w25qxx.h"
#include "mpu6050.h" //陀螺仪和加速度计
#include "ak8975.h" //磁力计
#include "ms5611.h"  //气压计
#include "optical_flow.h" //光流

/*
 * 飞控相关头文件
 */
#include "global.h"
#include "rc_channel.h"
#include "IMU.h"
#include "MAG.h"
#include "Parameters.h"
#include "GCS.h"

#include "utility.h"
#include "scheduler.h"
#include "BIT_MATH.h"
#include "filter.h"
#include "location.h"

class Copter
{
public:
	Copter();

    BIT_Scheduler scheduler;
    static const BIT_Scheduler::Task scheduler_tasks[];

    uint8_t setup(void);
    void loop(void);
    void loop_fast(void);

public:
    /// 任务调度表中的函数 都是静态函数
    /// 其他函数与该调度表函数分开
    static void update_MAG();
    static void update_AHRS();
    static void update_height();
    static void update_GPS();
    static void read_radio(void);

    static void update_flight_mode(void);
    static void attitude_rate_control(void);
    static void attitude_angle_control(void);
    static void height_control(void);
    static void position_control(void);
    static void navigate_control(void);

    static void arm_motors_check(void); //电机解锁检查
    static void write_motors(void); //电机输出
    
    static void write_data_to_flash(void); //将数据写入flash保存

    static void loop_100hz();
    static void loop_10hz();
    static void loop_1hz();
    static void end_of_task();

public:
    /// 初始化
    void driver_init(void); /// hardware init, this must be the first
    void Remote_Control_Init(void); // RC hardware init
    void rc_in_para_init(void); /// sets up rc channels from radio
    void all_pid_init(void);    //控制器所有PID参数初始化

public:
    /// copter.cpp
    void update_onboard_sensor_input(void); // 读取板载传感器数据 更新姿态数据	
    void update_flight_status(void); //更新飞行状态
    void update_external_sensor_switch(void); //更新外部传感器开关
	
    void ctrl_pid_rate_set(void);
    void ctrl_pid_angle_set(void);
    void ctrl_pid_height_set(void);
    void ctrl_pid_height_speed_set(void);
    void ctrl_pid_position_set(void);

    void unlock_check(u8 dT_ms); //解锁检测
    void height_data_update(void); //高度数据更新
    void height_data_fusion(u8 dT_ms,s32 fusion_acc_get,s32 ref_height_get);
    void WCZ_Data_Reset(void);
    void Center_Pos_Set(void);  //设置重心相对传感器的偏移量
    void auto_take_off_land_task(void); //自动起飞降落，目前推油门起飞用到自动起飞，自动降落还没有用
    void wait_ms(uint32_t ms);
    void wait_us(uint32_t us);

private:    
	// Global parameters are all contained within the 'g' class.
	Parameters g;

    /// 飞控计算所需要的全局变量
    int32_t baro_height;
    uint16_t ref_tof_height;
    int32_t baro_h_offset,ref_height_get_1,ref_height_get_2,ref_height_used;
    float acc_fusion_z; //z轴加速度融合数据
    float acc_fusion_x; //x轴加速度融合数据
    float acc_fusion_y; //y轴加速度融合数据 
    int32_t baro2tof_offset,tof2baro_offset;

    int16_t RC_CH[CH_NUM];
    
    //飞行状态所需变量
	float speed_set_h[VEC_XYZ];	
	float speed_set_h_norm[VEC_XYZ];
	float speed_set_h_norm_lpf[VEC_XYZ];

    //控制相关
    //位置控制
    float ctrl_target_position[VEC_XYZ];
    float ctrl_feedback_position[VEC_XYZ];    
    float ctrl_out_position[VEC_XYZ]; 
    
    //高度方向油门速率控制
    float auto_taking_off_speed;
    float ctrl_target_height_speed;
    float ctrl_feedback_height_speed;    
    float ctrl_out_height_speed;
    
    //角度控制
    float ctrl_target_angle[3];
    float ctrl_feedback_angle[3];    
    float ctrl_out_angle[3]; 
    float set_yaw_speed;
    float yaw_error;
    //角速率控制
    float ctrl_target_rate[3];
    float ctrl_feedback_rate[3];    
    float ctrl_out_rate[3];        

    int16_t motor_input[3];
    int16_t motor_input_throttle;
    int16_t motor_output[MOTORSNUM];
    
//	Vector3f omega; /// 陀螺仪的角速度	wzy这个可以借鉴
};

extern Copter copter;

#endif

