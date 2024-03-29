/*
 * rc_channel.cpp
 *
 *  Created on: 2018-3-15
 *      Author: wangbo
 */

#include <stdint.h>
#include <math.h>

#include "rc_channel.h"

#define ANGLE 0
#define RANGE 1

#define RC_CHANNEL_ANGLE 0
#define RC_CHANNEL_RANGE 1
#define RC_CHANNEL_ANGLE_RAW 2

//wzy
AP_RC_Channel::AP_RC_Channel()
{
    _high = 1;
    _filter = true;
    _reverse = 1;
    dead_zone = 0;
    scale_output = 1.0;
}

// setup the control preferences
void AP_RC_Channel::set_range(int low, int high)
{
	_type 	= RANGE;
	_high 	= high;
	_low 	= low;
}

void
AP_RC_Channel::set_angle(int angle)
{
	_type 	= ANGLE;
	_high 	= angle;
}

void
AP_RC_Channel::set_reverse(uint8_t reverse)
{
	if (reverse) _reverse = -1;
	else _reverse = 1;
}

bool
AP_RC_Channel::get_reverse(void)
{
	if (_reverse==-1) return 1;
	else return 0;
}

void
AP_RC_Channel::set_filter(bool filter)
{
	_filter = filter;
}

// call after first read
void
AP_RC_Channel::trim()
{
	radio_trim = radio_in;
}

// read input from APM_RC - create a control_in value
//因为pwm其实对应两种，1是舵机的有正反的有中立位，2是像油门这种是没有正反的，直接从最小到最大
//所以这个函数set_pwm意思是把接收机的信号1000～2000转换成-4500～+4500和 转换成 0～+1000这种
//
void
AP_RC_Channel::set_pwm(int pwm)
{
	radio_in = pwm;

	if(_type == RC_CHANNEL_RANGE)
	{
		control_in = pwm_to_range();
		control_in = (control_in < dead_zone) ? 0 : control_in;
	}
	else
	{
		control_in = pwm_to_angle();
	}
}

int
AP_RC_Channel::control_mix(float value)
{
	return (1 - fabs((float)control_in /(float) _high)) * value + control_in;
}

// are we below a threshold?
bool
AP_RC_Channel::get_failsafe(void)
{
	return (radio_in < (radio_min - 50));
}

// returns just the PWM without the offset from radio_min
//这个是把rc_channel的servo_out再次转换为1000～2000
//为什么需要转换呢，因为1，2，4的servo_out范围是-4500～4500
//而油门3的servo_out范围是0～1000
//我们要把servo_out转换成radio_out输出给电调
void
AP_RC_Channel::calc_pwm(void)
{
	if(_type == RC_CHANNEL_RANGE)
	{
		pwm_out         = range_to_pwm();//这个计算的结果是0～1000 _reverse初始化时是0，
		radio_out       = (_reverse >= 0) ? (radio_min + pwm_out) : (radio_max - pwm_out);
	}
	else if(_type == RC_CHANNEL_ANGLE_RAW)
	{
		pwm_out         = (float)servo_out * 0.1f;
		radio_out       = (pwm_out * _reverse) + radio_trim;
	}
	else
	{     // RC_CHANNEL_ANGLE
		pwm_out         = angle_to_pwm();//20170814这个计算的结果范围是-500～+500
		radio_out       = pwm_out + radio_trim;//这个又转为了1000～2000
	}
	radio_out = constrain(radio_out, radio_min, radio_max);
}

void
AP_RC_Channel::load_eeprom(void)
{
	//radio_min 	= eeprom_read_word((uint16_t *)	_address);
	//radio_max	= eeprom_read_word((uint16_t *)	(_address + 2));
	load_trim();
}

void
AP_RC_Channel::save_eeprom(void)
{
	//eeprom_write_word((uint16_t *)	_address, 			radio_min);
	//eeprom_write_word((uint16_t *)	(_address + 2), 	radio_max);
	save_trim();
}

void
AP_RC_Channel::save_trim(void)
{
	//eeprom_write_word((uint16_t *)	(_address + 4), 	radio_trim);
	//_ee.write_int((_address + 4), 	radio_trim);
}

void
AP_RC_Channel::load_trim(void)
{
	//radio_trim 	= eeprom_read_word((uint16_t *)	(_address + 4));
	//_ee.write_int((_address + 4), 	radio_trim);
}

void
AP_RC_Channel::zero_min_max()
{
	radio_min = radio_max = radio_in;
}

void
AP_RC_Channel::update_min_max()
{
	//radio_min = min(radio_min, radio_in);
	//radio_max = max(radio_max, radio_in);
}

int16_t
AP_RC_Channel::pwm_to_angle()
{
	int16_t radio_trim_high = radio_trim + dead_zone;//deadzone 60 radio_trim 1500
	int16_t radio_trim_low  = radio_trim - dead_zone;

	// prevent div by 0
	if ((radio_trim_low - radio_min) == 0 || (radio_max - radio_trim_high) == 0)
		return 0;

	if(radio_in < radio_trim)
		return  ((long)_high * (long)(radio_in - radio_trim)) / (long)(radio_trim - radio_min);//high 4500   radio_min 1000
	else
		return  ((long)_high * (long)(radio_in - radio_trim)) / (long)(radio_max  - radio_trim);
}

int16_t
AP_RC_Channel::angle_to_pwm()
{
	if(servo_out > 0)
	{
		return ((long)servo_out * (long)(radio_max - radio_trim)) / (long)_high;//servout*(2000-1500)/4500
	}
	else
		return ((long)servo_out * (long)(radio_trim - radio_min)) / (long)_high;
}

int16_t
AP_RC_Channel::pwm_to_range()
{
	/*
	 * rc_3也就是油门，在最初初始化的时候，最大最小值应该是跟地面站连接的时候
	 * 确定遥控器的输入最大最小值，但是我这里还没有地面站，所以先在初始化过程中设置为1000～2000了
	 */
	 int16_t r_in = constrain(radio_in, radio_min, radio_max);

	if (_reverse == -1)
	{
		r_in = radio_max - (r_in - radio_min);
	}

	int16_t radio_trim_low  = radio_min + dead_zone;

	/*
	 * _low是在g.channel_throttle.set_range(0, 1000);时设置的，_low=0,_high=1000
	 * 所以这个pwm_to_range函数最终返回的范围是0~+1000
	 */
	if(r_in > radio_trim_low)
		return (_low + ((long)(_high - _low) * (long)(r_in - radio_trim_low)) / (long)(radio_max - radio_trim_low));
	else if(dead_zone > 0)
		return 0;
	else
		return _low;//这个有可能使负数呀
}

int16_t
AP_RC_Channel::range_to_pwm()
{
	return ((long)(servo_out - _low) * (long)(radio_max - radio_min)) / (long)(_high - _low);
}

float
AP_RC_Channel::norm_input()
{
	if(radio_in < radio_trim)
		return _reverse * (float)(radio_in - radio_trim) / (float)(radio_trim - radio_min);
	else
		return _reverse * (float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim);
}

float
AP_RC_Channel::norm_output()
{
	if(radio_out < radio_trim)
		return (float)(radio_out - radio_trim) / (float)(radio_trim - radio_min);
	else
		return (float)(radio_out - radio_trim) / (float)(radio_max  - radio_trim);
}

float
AP_RC_Channel::constrain(float m,float a,float b)
{
	if(m<=a)        m=a;
	else if(m>=b)   m=b;

	return m;
}
