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
    Parameter_save.value.reset_ok = 1; //�������óɹ�
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
//---	��̬���ƽ��ٶȻ�PID����
	Parameter_save.value.pid_attitude_rate_save[ROLL][KP]  = 4.0f; 
	Parameter_save.value.pid_attitude_rate_save[ROLL][KI]  = 2.0f; 
	Parameter_save.value.pid_attitude_rate_save[ROLL][KD]  = 0.2f; 	
	Parameter_save.value.pid_attitude_rate_save[PITCH][KP] = 4.0f; 
	Parameter_save.value.pid_attitude_rate_save[PITCH][KI] = 2.0f; 
	Parameter_save.value.pid_attitude_rate_save[PITCH][KD] = 0.2f; 
	Parameter_save.value.pid_attitude_rate_save[YAW][KP]   = 6.0f; 
	Parameter_save.value.pid_attitude_rate_save[YAW][KI]   = 1.0f; 
	Parameter_save.value.pid_attitude_rate_save[YAW][KD]   = 0.2f; 
//---	��̬���ƽǶȻ�PID����
	Parameter_save.value.pid_attitude_angle_save[ROLL][KP]  = 6.0f; 
	Parameter_save.value.pid_attitude_angle_save[ROLL][KI]  = 0.0f; 
	Parameter_save.value.pid_attitude_angle_save[ROLL][KD]  = 0.0f; 
	Parameter_save.value.pid_attitude_angle_save[PITCH][KP] = 6.0f; 
	Parameter_save.value.pid_attitude_angle_save[PITCH][KI] = 0.0f;
	Parameter_save.value.pid_attitude_angle_save[PITCH][KD] = 0.0f; 
	Parameter_save.value.pid_attitude_angle_save[YAW][KP]   = 4.0f; 
	Parameter_save.value.pid_attitude_angle_save[YAW][KI]   = 0.0f; 
	Parameter_save.value.pid_attitude_angle_save[YAW][KD]   = 0.5f; 
//---	�߶ȷ����������ʻ�PID����	
	Parameter_save.value.pid_height_speed_save[KP] = 2.0f;  
	Parameter_save.value.pid_height_speed_save[KI] = 1.0f;     
	Parameter_save.value.pid_height_speed_save[KD] = 0.05f;    
//---	�߶ȿ��Ƹ߶Ȼ�PID����
	Parameter_save.value.pid_position_save[Z][KP] = 1.0f;           
	Parameter_save.value.pid_position_save[Z][KI] = 0.0f;        
	Parameter_save.value.pid_position_save[Z][KD] = 0.0f;         
}

void Parameter_Write(void)
{
	copter.all_pid_init();	//�洢PID���������³�ʼ��PID	
	Parameter_save.value.reset_ok = 1; //����� �����в�������

	Parameter_Copy_Autopilot_To_Flash();
	
	Flash_SectorErase(0x000000, 1);						//������һ����
	Flash_SectorsWrite(0x000000, &Parameter_save.byte[0], 1);	//������д���һ����
}

void Parameter_Read(void)
{
	Flash_SectorsRead(0x000000, &Parameter_save.byte[0], 1); //��ȡ��һ�����ڵĲ���
	
	if(Parameter_save.value.reset_ok != 1)	//Flash��û�в���ʱ����Ҫ����һ�β������ú�д��
	{		
		Parameter_Reset();
		PID_Reset();
		Parameter_Write();
	}	
	Parameter_Copy_Flash_To_Autopilot();
}

void Copter::write_data_to_flash(void)
{
	/*��Ϊд��flash��ʱ�ϳ������Ƿɿ�����һ�������߼����ڽ������ǲ����в���д��ģ���ʱ����һ����Ҫд���־���ȷɻ�������������д��������������а�ȫ��
	Ϊ�˱������������������������flashд�����Σ����Ƿɿؼ���һ����ʱ�߼��������ı��һ�룬�Ž���д�����������һ��д��������������flash��д����
    */
    uint16_t dT_ms = 500; // 500ms����
	if(para_sta.save_en )				//����洢
	{
		if(para_sta.save_trig == 1) 	//��������洢���1
		{
			LED_STA.saving = 1;
			
			para_sta.time_delay = 0;  	//��ʱ��λ
			para_sta.save_trig = 2;   	//�����洢���2
		}
		
		if(para_sta.save_trig == 2) 	//��������洢���2
		{
			if(para_sta.time_delay < 3000) //��ʱС��3000ms
			{
				para_sta.time_delay += dT_ms; //��ʱ
			}
			else
			{
				
				para_sta.save_trig = 0;  //�洢��Ǹ�λ
				Parameter_Write();      //ִ�д洢
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
