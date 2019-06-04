/*
 * rc.cpp
 *
 *  Created on: 2018-3-15
 *      Author: wangbo
 */

#include <stdint.h>

#include "rc.h"

#include "all_external_device.h"

#include "global.h"  //test 测试限制油门最大值

AP_RC::AP_RC()
{

}

void
AP_RC::init()
{



	/*
	 * 在我看来初始化，就应该是下面的这些set_ch_pwm呀，
	 * 但是他们为什么用的上面了，是为了封装吗？
	 */
	/*
	set_ch_pwm(0, 1500);
	set_ch_pwm(1, 1500);
	set_ch_pwm(2, 1500);
	set_ch_pwm(3, 1500);
	*/

}


void
AP_RC::output_ch_pwm(uint8_t ch, uint16_t pwm)
{

	switch(ch)
	{
	case CH_1:
		all_external_device_output.rc_raw_out_0=(float)pwm;
		break;
	case CH_2:
		all_external_device_output.rc_raw_out_1=(float)pwm;
		break;
	case CH_3:
		all_external_device_output.rc_raw_out_2=(float)pwm;
		break;
	case CH_4:
		all_external_device_output.rc_raw_out_3=(float)pwm;
		break;
	}
}


