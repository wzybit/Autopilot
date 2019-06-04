/*
 * Parameters.h
 *
 *  Created on: 2017-9-30
 *      Author: wangbo
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "global.h"
#include "rc_channel.h"
#include "pid.h"

/*
 * 这个文件需要定义Parameter这个结构变量
 * 同时包含所有的
 * 宏定义的值放在哪个文件呢？
 * 把宏定义都放在global.h文件吧，这个文件是我自己根据我自己的习惯建立的，跟apm没有关系
 * 主要是为了在整个工程中除了库文件都可以用到，进行值的传递或者仿真
 */
class Parameters
{
public:   
	// RC channels wzy 没有用APM的通道，因为数量级不同之后还要调参数
	AP_RC_Channel  channel_roll;
	AP_RC_Channel  channel_pitch;
	AP_RC_Channel  channel_throttle;
	AP_RC_Channel  channel_rudder;
	AP_RC_Channel  channel_5;
	AP_RC_Channel  channel_6;
	AP_RC_Channel  channel_7;
	AP_RC_Channel  channel_8;

	// BIT_PID controllers
    BIT_PID  pid_position_fix[VEC_XYZ]; //位置控制位置修正环PID参数 wzy这个其实以后可以修改
    BIT_PID  pid_position[VEC_XYZ];     //位置控制位置环PID参数
    BIT_PID  pid_height_speed;          //高度方向油门速率环PID参数
    BIT_PID  pid_attitude_angle[3];     //姿态控制角速度环PID参数
	BIT_PID  pid_attitude_rate[3];      //姿态控制角度环PID参数
};

__packed struct Parameter_save_s
{
	uint8_t  reset_ok;	    //Flash里没有参数时，需要进行一次参数重置和写入
	uint8_t	 rcInMode;		//接收机模式，分别为PWM型 PPM型 SBUS型

	float    acc_offset[VEC_XYZ];  	 //加速度计零偏
	float    gyro_offset[VEC_XYZ]; 	 //陀螺仪零偏
	float    center_pos_cm[VEC_XYZ]; //重心相对传感器位置偏移量
	float    mag_offset[VEC_XYZ];  	 //磁力计零偏
	float    mag_gain[VEC_XYZ];    	 //磁力计校正比例
	
    float    pid_attitude_angle_save[3][PID]; //姿态控制角度环PID参数 
	float    pid_attitude_rate_save[3][PID];  //姿态控制角速度环PID参数
    float    pid_position_save[VEC_XYZ][PID]; //位置控制位置环PID参数 
    float    pid_height_speed_save[PID];      //高度方向油门速率环PID参数
    
	float    auto_take_off_speed;   //自动起飞速度
    float    auto_take_off_height;  //自动起飞高度
    float    auto_landing_speed;    //自动降落速度
    int16_t  motor_idle_speed;    //解锁后怠速启动
};

union Parameter_save_u
{
	//这里使用联合体，长度是4KByte，联合体内部是一个结构体，该结构体内是需要保存的参数 wzy用联合体是为了读写扇区方便
	struct Parameter_save_s value;
	uint8_t byte[4096];
};
extern union Parameter_save_u Parameter_save;

typedef struct
{
	uint8_t save_en;
	uint8_t save_trig;
	uint16_t time_delay;
}parameter_state_st ;
extern parameter_state_st para_sta;

void data_save(void);
void Parameter_Read(void);

#endif /* PARAMETERS_H_ */
