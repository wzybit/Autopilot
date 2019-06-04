/*
 * pid.h
 *
 *  Created on: 2016年5月10日
 *      Author: wangbo
 */

#ifndef HEADERS_PID_H_
#define HEADERS_PID_H_

/***把PID写成类*************/

#include <inttypes.h>
#include <math.h>		// for fabs()

//实例化的对象只对应(管理)一个 PID 控制器
class BIT_PID
{
public:
	BIT_PID();

	float 	      get_pid(float error, float dt_ms, float scaler = 1.0);

	float         get_pi(float error, float dt);
	float         get_p(float error);
	float         get_i(float error, float dt);
	float         get_d(float input, float dt);

	/*
	 * 清零积分量
	 */
	void	reset_I();

	/*
	 * 设置PID控制器的参数
	 */
	void	set_kP(const float v)		{ _kp = v; }
	void	set_kI(const float v)		{ _ki = v; }
	void	set_kD(const float v)		{ _kd = v; }
    void	set_kd_expect(const float v)		{ _kd_expect = v; }
	void	set_imax(const float v)	{ _imax = v; }

	/*
	 * 获取PID控制器的参数
	 */
	float	get_kP()			{ return _kp; }
	float	get_kI()			{ return _ki; }
	float	get_kD()			{ return _kd; }
	float	get_imax()			{ return _imax; }

    void    set_integrator(float i)
    {
        _integrator = i;
    }
	float	get_integrator() const	{ return _integrator; }

	static const float        _filter ;//= 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";

private:
	float				_kp;
	float				_ki;
	float				_kd;
    float               _kd_expect;

	float			_integrator;	//积分
    float           _derivative;    //微分 
    float           _error_expect;  //期望值误差
    float           _error_feedback; //反馈值误差
    float           _last_expect;   //上次的期望值
    float           _last_feedback; //上次的反馈值
	float			_last_error;		///< last error for derivative
	float			_last_derivative; 	///< last derivative for low-pass filter
	

	/*
	 * 积分限幅
	 */
	//static const float _imax = 0.174*3;//这个是积分幅度的最大值，限制在10度，10/180*3.14=0.174
	float _imax ;//这个是积分幅度的最大值，限制在10度，10/180*3.14=0.174

	/*
	 * 低通滤波器
	 */
	/// Low pass filter cut frequency for derivative calculation.
	///
    //static const float        _filter= 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
	// Examples for _filter:
	// f_cut = 10 Hz -> _filter = 15.9155e-3
	// f_cut = 15 Hz -> _filter = 10.6103e-3
	// f_cut = 20 Hz -> _filter =  7.9577e-3
	// f_cut = 25 Hz -> _filter =  6.3662e-3
	// f_cut = 30 Hz -> _filter =  5.3052e-3

	/// Low pass filter cut frequency for derivative calculation.
	///
	/// 20 Hz becasue anything over that is probably noise, see
	/// http://en.wikipedia.org/wiki/Low-pass_filter.
	///
	static const uint8_t _fCut = 20;
};

#endif /* HEADERS_PID_H_ */
