/*
 * motors.cpp
 *
 *  Created on: 2017-10-1
 *      Author: wangbo
 */

#include "Copter.h"

/*
四轴：
      机头
   2       1
     \   /
      \ /
      / \
     /   \
   3       4
      屁股
*/
void Copter::write_motors()
{
    u8 dT_ms = 2; //2ms task
    int16_t idle_speed;
    static uint16_t motor_prepara_cnt;
    static int16_t motor_step[MOTORSNUM];
    
    if(globalFlag.unlockStatus)
	{		
		idle_speed = 10 * LIMIT(Parameter_save.value.motor_idle_speed, 0, 30);
		
		if(globalFlag.motor_preparation == 0)
		{
			motor_prepara_cnt += dT_ms;
			
			if(globalFlag.motor_preparation == 0)
			{			
				if(motor_prepara_cnt<300)
				{
					copter.motor_output[0] = idle_speed;
				}
				else if(motor_prepara_cnt<600)
				{
					copter.motor_output[1] = idle_speed;
				}
				else if(motor_prepara_cnt<900)
				{
					copter.motor_output[2] = idle_speed;
				}	
				else if(motor_prepara_cnt<1200)
				{	
					copter.motor_output[3] = idle_speed;
				}
				else
				{
					globalFlag.motor_preparation = 1;
					motor_prepara_cnt = 0;
				}
			}
		}	
	}
	else
	{
		globalFlag.motor_preparation = 0;
	}
			
	if(globalFlag.motor_preparation == 1)
	{	
		motor_step[0] = copter.motor_input_throttle + copter.motor_input[YAW] - copter.motor_input[ROLL] + copter.motor_input[PITCH];
		motor_step[1] = copter.motor_input_throttle - copter.motor_input[YAW] + copter.motor_input[ROLL] + copter.motor_input[PITCH];
		motor_step[2] = copter.motor_input_throttle + copter.motor_input[YAW] + copter.motor_input[ROLL] - copter.motor_input[PITCH];
		motor_step[3] = copter.motor_input_throttle - copter.motor_input[YAW] - copter.motor_input[ROLL] - copter.motor_input[PITCH];
		
		for(uint8_t i = 0; i < MOTORSNUM; i++)
		{	
			motor_step[i] = LIMIT(motor_step[i], idle_speed, 1000);
		}
	}
	
	for(uint8_t i = 0; i < MOTORSNUM; i++)
	{
		if(globalFlag.unlockStatus)
		{
			if(globalFlag.motor_preparation == 1)
			{
				copter.motor_output[i] = LIMIT(motor_step[i], idle_speed, 1000);
			}
		}
		else
		{		
			copter.motor_output[i] = 0;
		}	
	}

	write_pwm_all(copter.motor_output);
}
