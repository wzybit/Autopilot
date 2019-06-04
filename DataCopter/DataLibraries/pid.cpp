/*
 * pid.c
 *
 *  Created on: 2016年5月10日
 *      Author: wangbo
 */

#include <stdio.h>
#include <math.h>

#include "pid.h"
#include "global.h"

BIT_PID::BIT_PID() {}

const float        BIT_PID::_filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";

float BIT_PID::get_pid(float dt_s, float expect, float feedback)
{
    float error = 0;
	float output = 0;
    float hz;
	hz = safe_div(1.0f, dt_s, 0);
    
    //计算微分值
	_error_expect = (expect - _last_expect) * hz;	
	_error_feedback = (feedback - _last_feedback) * hz;	
	_derivative = _kd_expect * _error_expect - _kd * _error_feedback;
	
    //计算偏差值
	error = expect - feedback;	

    //计算积分值
	_integrator += _ki * error * dt_s;
	_integrator = LIMIT(_integrator, -_imax, _imax);
	
	output = _kp * error + _derivative + _integrator;
	
	_last_feedback = feedback;
	_last_expect = expect; 

	return output;
}

void
BIT_PID::reset_I()
{
	_integrator = 0;
	_last_error = 0;
	_last_derivative = 0;
}

float BIT_PID::get_p(float error)
{
    return (float)error * _kp;
}

float BIT_PID::get_i(float error, float dt)
{
    if(dt != 0)
    {
        _integrator += ((float)error * _ki) * dt;

        if (_integrator < -_imax)
        {
            _integrator = -_imax;
        }
        else if (_integrator > _imax)
        {
            _integrator = _imax;
        }
    }

    return _integrator;
}

float BIT_PID:: get_d(float input, float dt)
{
	if ((_kd != 0) && (dt != 0))
	{
		if (isnan(_last_derivative)) {
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change			
			_derivative = 0;
			_last_derivative = 0;
		} else {
			// calculate instantaneous derivative
		_derivative = (input - _last_error) / dt;
		}

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		_derivative = _last_derivative +
				               (dt / ( _filter + dt)) * (_derivative - _last_derivative);

		// update state
		_last_error            = input;
		_last_derivative    = _derivative;

		// add in derivative component
		return _kd * _derivative;
	}

	return 0;
}

float BIT_PID::get_pi(float error, float dt)
{
    return get_p(error) + get_i(error, dt);
}
