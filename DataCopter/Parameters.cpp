/*
 * Parameters.cpp
 *
 *  Created on: 2017-9-30
 *      Author: wangbo
 */

#include <stdio.h>
#include "Parameters.h"
#include "copter.h"
#include "IMU.h"

parameter_state_st para_sta;
union Parameter_save_u Parameter_save;

void data_save(void)
{
	para_sta.save_en = !globalFlag.unlockStatus;
	para_sta.save_trig = 1;
}


void Parameter_Copy_Flash_To_Autopilot()
{
    globalConfig.rcInMode = Parameter_save.value.rcInMode;
	for(u8 i = 0;i<3;i++)
	{	
		globalSaveData.acc_offset[i]	=	Parameter_save.value.acc_offset[i];
		globalSaveData.gyro_offset[i]	=	Parameter_save.value.gyro_offset[i];
		globalSaveData.mag_offset[i]	=	Parameter_save.value.mag_offset[i];  
		globalSaveData.mag_gain[i]		=	Parameter_save.value.mag_gain[i];  			
	}		
	copter.Center_Pos_Set();   
}

void Parameter_Copy_Autopilot_To_Flash()
{
	for(u8 i = 0;i<3;i++)
	{	
		Parameter_save.value.acc_offset[i]	=	globalSaveData.acc_offset[i];
		Parameter_save.value.gyro_offset[i]	=	globalSaveData.gyro_offset[i];
		Parameter_save.value.mag_offset[i]	=	globalSaveData.mag_offset[i];  
		Parameter_save.value.mag_gain[i]	=	globalSaveData.mag_gain[i];   
	}
}

void Parameter_Reset(void)
{	
    Parameter_save.value.reset_ok = 1; //参数重置成功
	Parameter_save.value.rcInMode = PWM;
    
	for(u8 i = 0;i<3;i++)
	{
		Parameter_save.value.acc_offset[i]    = 0;
		Parameter_save.value.gyro_offset[i]   = 0;
		Parameter_save.value.center_pos_cm[i] = 0;
		Parameter_save.value.mag_offset[i]    = 0;  
		Parameter_save.value.mag_gain[i]      = 1;    
	}
    
	Parameter_save.value.auto_take_off_height = 0.0f;  //cm
	Parameter_save.value.auto_take_off_speed  = 150.0f;
	Parameter_save.value.auto_landing_speed   = 60.0f;	
	Parameter_save.value.motor_idle_speed = 20;//20%
	
	Parameter_Copy_Flash_To_Autopilot();
}

void PID_Reset()
{
//---	姿态控制角速度环PID参数
	Parameter_save.value.pid_attitude_rate_save[ROLL][KP]  = 4.0f; 
	Parameter_save.value.pid_attitude_rate_save[ROLL][KI]  = 2.0f; 
	Parameter_save.value.pid_attitude_rate_save[ROLL][KD]  = 0.2f; 	
	Parameter_save.value.pid_attitude_rate_save[PITCH][KP] = 4.0f; 
	Parameter_save.value.pid_attitude_rate_save[PITCH][KI] = 2.0f; 
	Parameter_save.value.pid_attitude_rate_save[PITCH][KD] = 0.2f; 
	Parameter_save.value.pid_attitude_rate_save[YAW][KP]   = 6.0f; 
	Parameter_save.value.pid_attitude_rate_save[YAW][KI]   = 1.0f; 
	Parameter_save.value.pid_attitude_rate_save[YAW][KD]   = 0.2f; 
//---	姿态控制角度环PID参数
	Parameter_save.value.pid_attitude_angle_save[ROLL][KP]  = 6.0f; 
	Parameter_save.value.pid_attitude_angle_save[ROLL][KI]  = 0.0f; 
	Parameter_save.value.pid_attitude_angle_save[ROLL][KD]  = 0.0f; 
	Parameter_save.value.pid_attitude_angle_save[PITCH][KP] = 6.0f; 
	Parameter_save.value.pid_attitude_angle_save[PITCH][KI] = 0.0f;
	Parameter_save.value.pid_attitude_angle_save[PITCH][KD] = 0.0f; 
	Parameter_save.value.pid_attitude_angle_save[YAW][KP]   = 4.0f; 
	Parameter_save.value.pid_attitude_angle_save[YAW][KI]   = 0.0f; 
	Parameter_save.value.pid_attitude_angle_save[YAW][KD]   = 0.5f; 
//---	高度方向油门速率环PID参数	
	Parameter_save.value.pid_height_speed_save[KP] = 2.0f;  
	Parameter_save.value.pid_height_speed_save[KI] = 1.0f;     
	Parameter_save.value.pid_height_speed_save[KD] = 0.05f;    
//---	高度控制高度环PID参数
	Parameter_save.value.pid_position_save[Z][KP] = 1.0f;           
	Parameter_save.value.pid_position_save[Z][KI] = 0.0f;        
	Parameter_save.value.pid_position_save[Z][KD] = 0.0f;         
}

void Parameter_Write(void)
{
	copter.all_pid_init();	//存储PID参数后，重新初始化PID	
	Parameter_save.value.reset_ok = 1; //存过后 不进行参数重置

	Parameter_Copy_Autopilot_To_Flash();
	
	Flash_SectorErase(0x000000, 1);						//擦除第一扇区
	Flash_SectorsWrite(0x000000, &Parameter_save.byte[0], 1);	//将参数写入第一扇区
}

void Parameter_Read(void)
{
	Flash_SectorsRead(0x000000, &Parameter_save.byte[0], 1); //读取第一扇区内的参数
	
	if(Parameter_save.value.reset_ok != 1)	//Flash里没有参数时，需要进行一次参数重置和写入
	{		
		Parameter_Reset();
		PID_Reset();
		Parameter_Write();
	}	
	Parameter_Copy_Flash_To_Autopilot();
}

void Copter::write_data_to_flash(void)
{
	/*因为写入flash耗时较长，我们飞控做了一个特殊逻辑，在解锁后，是不进行参数写入的，此时会置一个需要写入标志，等飞机降落锁定后，再写入参数，提升飞行安全性
	为了避免连续更新两个参数，造成flash写入两次，我们飞控加入一个延时逻辑，参数改变后一秒，才进行写入操作，可以一次写入多项参数，降低flash擦写次数
    */
    uint16_t dT_ms = 500; // 500ms任务
	if(para_sta.save_en )				//允许存储
	{
		if(para_sta.save_trig == 1) 	//如果触发存储标记1
		{
			LED_STA.saving = 1;
			
			para_sta.time_delay = 0;  	//计时复位
			para_sta.save_trig = 2;   	//触发存储标记2
		}
		
		if(para_sta.save_trig == 2) 	//如果触发存储标记2
		{
			if(para_sta.time_delay < 3000) //计时小于3000ms
			{
				para_sta.time_delay += dT_ms; //计时
			}
			else
			{
				
				para_sta.save_trig = 0;  //存储标记复位
				Parameter_Write();      //执行存储
				GCS_SendString("Set save OK!");
				LED_STA.saving = 0;
			}
		}
		else
		{
			para_sta.time_delay = 0;
		}
		
	}
	else
	{
		para_sta.time_delay = 0;
		para_sta.save_trig = 0;
	}
}
