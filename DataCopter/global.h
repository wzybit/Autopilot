/*
 * global.h
 *
 *  Created on: 2018-3-12
 *      Author: wangbo
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <stdint.h>

/// 系统时钟 计数 数据类型
#define TICK_PER_SECOND	1000
#define US_PER_TICK	((uint64_t)(1e6 / TICK_PER_SECOND))  // micro second per tick

//#define DEBUG_SWITCH
#ifdef  DEBUG_SWITCH
#define DEBUG_PRINTF(fmt,args...) printf(fmt, ##args)
#else
#define DEBUG_PRINTF(fmt,args...) /*do nothing */
#endif

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define my_pow(a) ((a)*(a))
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )
#define my_sign(x) ( ((x) > 1e-6f) ? 1:( ((x)<-1e-6f) ? -1 : 0) )
#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))	

#define GCS_USE_USART 				//开启串口数传上位机功能
#define GCS_USE_USB_HID				//开启飞控USBHID连接上位机功能

/// 中断优先级
//Group_3: 3 bits for pre-emption priority    1 bits for subpriority 
#define NVIC_GROUP NVIC_PriorityGroup_3		//中断分组选择

#define NVIC_PWMIN_PRE	  1			//接收机PWM采集中断配置
#define NVIC_PWMIN_SUB	  0

#define NVIC_TIME_PRE     1			//定时器中断配置 暂未使用
#define NVIC_TIME_SUB     1

#define NVIC_USART1_PRE	  2			//串口1中断优先级配置 暂未使用
#define NVIC_USART1_SUB	  0

#define NVIC_USART2_PRE	  2			//串口2中断优先级配置 SBUS
#define NVIC_USART2_SUB	  1

#define NVIC_USART3_PRE   3			//串口3中断优先级配置 可用作GCS通信
#define NVIC_USART3_SUB	  0

#define NVIC_UART4_PRE	  3			//串口4中断优先级配置 暂未使用
#define NVIC_UART4_SUB	  1

#define NVIC_UART5_PRE	  4			//串口5中断优先级配置 暂未使用
#define NVIC_UART5_SUB	  0

#define NVIC_USART6_PRE	  4			//串口6中断优先级配置 暂未使用
#define NVIC_USART6_SUB	  1

#define SBUS_IN_MIN 352.0f  /// futaba T14SG遥控器的sbus解析后，最小的数值
#define SBUS_IN_MAX 1696.0f /// futaba T14SG遥控器的sbus解析后，最大的数值

#define MOTORSNUM 4  //电机数目 4 6 8

#define OFFSET_AV_NUM 50  //IMU传感器数据校准得出偏移量时用到的数据量
//==
//GYRO_ACC_FILTER
//==
//500KV以下 0.1f~0.15f
//500~2000KV 0.15f~0.3f
//2000kv以上 0.3f-0.8f
//==
#define GYRO_ACC_FILTER 0.15f //
#define RANGE_PN2000_TO_RAD 0.001065f
#define RANGE_PN8G_TO_CMSS  0.2395f
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	角度转弧度

#define MAX_XY_SPEED 500.0f //最大水平速度   厘米每秒 cm/s
#define MAX_Z_SPEED_UP 350.0f //z轴上升最大速度     厘米每秒 cm/s ? wzy感觉这个和飞控质量有关
#define MAX_Z_SPEED_DW 250.0f //z轴下降最大速度

#define MAX_CTRL_ANGLE     25.0f

#define THR_INTE_LIM_SET   70.0f  //油门积分百分比 % 
#define MOTOR_OUT_P 	   0.35f  //电机输出量比例
#define THR_INTE_LIM   THR_INTE_LIM_SET/MOTOR_OUT_P 
#define THR_START      35.0f  //油门起调量百分比 %
#define MAX_THR_SET    85.0f  //最大油门百分比 %

#define MOTOR_ESC_TYPE 1  //2：无刷电机带刹车的电调，1：无刷电机不带刹车的电调

#define  SCALE_P_ENLARGE  1000.0f
#define  SCALE_I_ENLARGE  1000.0f
#define  SCALE_D_ENLARGE  1000.0f

#define  SCALE_P_RATE_ENLARGE   1000.0f
#define  SCALE_I_RATE_ENLARGE   1000.0f
#define  SCALE_D_RATE_ENLARGE   1000.0f

#define  SCALE_P_RESTORE   (1.0f / SCALE_P_ENLARGE)
#define  SCALE_I_RESTORE   (1.0f / SCALE_I_ENLARGE)
#define  SCALE_D_RESTORE   (1.0f / SCALE_D_ENLARGE)

#define  SCALE_P_RATE_RESTORE   (1.0f / SCALE_P_RATE_ENLARGE)
#define  SCALE_I_RATE_RESTORE   (1.0f / SCALE_I_RATE_ENLARGE)
#define  SCALE_D_RATE_RESTORE   (1.0f / SCALE_D_RATE_ENLARGE)

enum Control_Mode
{
    STABILIZE   = 0, //增稳
    LOC_HOLD    = 1, //定点悬停
    RETURN_HOME = 2, //返航
};

enum
{
	AUTO_TAKE_OFF_NULL = 0,
	AUTO_TAKE_OFF = 1,
	AUTO_TAKE_OFF_FINISH,
	AUTO_LAND,
};

enum E_RCInMode
{
    PWM = 0,
    PPM,
    SBUS,
};

enum I2C_Mode
{
    I2C_1 = 0,
    I2C_2,
    I2C_3,
};

enum
{
    CH_ROL = 0,
    CH_PIT ,
    CH_THR ,
    CH_YAW ,
    AUX1 ,
    AUX2 ,
    AUX3 ,
    AUX4 ,
    CH_NUM,//8
}; //遥控器输入通道

enum
{
	X = 0,
	Y = 1,
	Z = 2,
	VEC_XYZ,
};

enum
{
	ROLL = 0,
	PITCH = 1,
	YAW = 2,
};

enum
{
	KP = 0,
	KI = 1,
	KD = 2,
	PID,
};

enum
{
    A_X = 0,
    A_Y ,
    A_Z ,
    G_X , //之前旧版代码这里问题很大 X和Y颠倒了
    G_Y ,
    G_Z ,
    TEM ,
    ITEMS ,
};

typedef struct 
{
    float x;
    float y;
    float z;
}xyz_f_t;

typedef struct 
{
    int16_t x;
    int16_t y;
    int16_t z;
}xyz_s16_t;

struct T_GLOBAL_TIME
{
    uint64_t system_tick_update_s; // tick++ per second
    uint64_t system_tick_update_ms; // tick++ per milli second   
    uint64_t time_us; // 系统的微秒计数 64bit
    uint32_t time_loop_fast_us; //系统循环里 快循环使用的时间 单位us
    uint32_t time_loop_scheduler_sum_us; //系统循环里 任务调度表内任务使用的总时间 单位us
    uint32_t time_loop_scheduler_us[100]; //系统循环里 任务调度表内每个任务使用的时间 单位us 这个最多限制100个任务
};  

struct T_GLOBAL_FLAG
{
    uint8_t power_ok;
    uint8_t init_ok;
    uint8_t sensor_imu_ok;
    uint8_t mems_temperature_ok; //惯性传感器加热开关 这里直接置1
    
    //传感器状态
    uint8_t gyro_ok;
    uint8_t acc_ok;
    uint8_t mag_ok;
    uint8_t baro_ok;
    uint8_t tof_ok; //飞控外接激光测距
    uint8_t of_ok; //光流
    
    //控制开关 是否使用
    uint8_t gps_on;
    uint8_t of_flow_on;  //光流信息
    uint8_t of_tof_on;  //光流激光数据
    uint8_t tof_on;  //飞控外接激光测距
    
    //控制指令
    uint8_t unlock_cmd;//解锁指令 可以由遥控器、地面站或降落等触发
    uint8_t locking; //遥控器摇杆处于解锁状态
    
    //飞行状态
    uint8_t unlockStatus; //解锁状态
    uint8_t motor_preparation; //解锁后电机需要准备一小段时间
    uint8_t motionless; 
    uint8_t taking_off; //起飞
    uint8_t flying; //飞行 wzy 起飞和飞行状态注意区分 
    uint8_t auto_take_off_land; //自动起飞和自动降落标志位
    uint8_t rc_loss_back_home; //丢失遥控器信号 返航标志
};

struct T_GLOBAL_CONFIG
{
    uint8_t rcInMode;
    uint8_t flightMode;
};

struct T_GLOBAL_SAVE_DATA
{
	float acc_offset[VEC_XYZ]; //加速度计零偏
	float gyro_offset[VEC_XYZ]; //陀螺仪零偏
	float center_pos_cm[VEC_XYZ]; //重心相对传感器位置偏移量	
	float mag_offset[VEC_XYZ]; //磁力计零偏
	float mag_gain[VEC_XYZ];  //磁力计校正比例
};  

extern struct T_GLOBAL_TIME globalSystemTime;
extern struct T_GLOBAL_FLAG globalFlag;
extern struct T_GLOBAL_CONFIG globalConfig;
extern struct T_GLOBAL_SAVE_DATA globalSaveData;

#endif /* GLOBAL_H_ */
