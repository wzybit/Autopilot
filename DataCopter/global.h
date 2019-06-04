/*
 * global.h
 *
 *  Created on: 2018-3-12
 *      Author: wangbo
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <stdint.h>

/// ϵͳʱ�� ���� ��������
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

#define GCS_USE_USART 				//��������������λ������
#define GCS_USE_USB_HID				//�����ɿ�USBHID������λ������

/// �ж����ȼ�
//Group_3: 3 bits for pre-emption priority    1 bits for subpriority 
#define NVIC_GROUP NVIC_PriorityGroup_3		//�жϷ���ѡ��

#define NVIC_PWMIN_PRE	  1			//���ջ�PWM�ɼ��ж�����
#define NVIC_PWMIN_SUB	  0

#define NVIC_TIME_PRE     1			//��ʱ���ж����� ��δʹ��
#define NVIC_TIME_SUB     1

#define NVIC_USART1_PRE	  2			//����1�ж����ȼ����� ��δʹ��
#define NVIC_USART1_SUB	  0

#define NVIC_USART2_PRE	  2			//����2�ж����ȼ����� SBUS
#define NVIC_USART2_SUB	  1

#define NVIC_USART3_PRE   3			//����3�ж����ȼ����� ������GCSͨ��
#define NVIC_USART3_SUB	  0

#define NVIC_UART4_PRE	  3			//����4�ж����ȼ����� ��δʹ��
#define NVIC_UART4_SUB	  1

#define NVIC_UART5_PRE	  4			//����5�ж����ȼ����� ��δʹ��
#define NVIC_UART5_SUB	  0

#define NVIC_USART6_PRE	  4			//����6�ж����ȼ����� ��δʹ��
#define NVIC_USART6_SUB	  1

#define SBUS_IN_MIN 352.0f  /// futaba T14SGң������sbus��������С����ֵ
#define SBUS_IN_MAX 1696.0f /// futaba T14SGң������sbus������������ֵ

#define MOTORSNUM 4  //�����Ŀ 4 6 8

#define OFFSET_AV_NUM 50  //IMU����������У׼�ó�ƫ����ʱ�õ���������
//==
//GYRO_ACC_FILTER
//==
//500KV���� 0.1f~0.15f
//500~2000KV 0.15f~0.3f
//2000kv���� 0.3f-0.8f
//==
#define GYRO_ACC_FILTER 0.15f //
#define RANGE_PN2000_TO_RAD 0.001065f
#define RANGE_PN8G_TO_CMSS  0.2395f
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	�Ƕ�ת����

#define MAX_XY_SPEED 500.0f //���ˮƽ�ٶ�   ����ÿ�� cm/s
#define MAX_Z_SPEED_UP 350.0f //z����������ٶ�     ����ÿ�� cm/s ? wzy�о�����ͷɿ������й�
#define MAX_Z_SPEED_DW 250.0f //z���½�����ٶ�

#define MAX_CTRL_ANGLE     25.0f

#define THR_INTE_LIM_SET   70.0f  //���Ż��ְٷֱ� % 
#define MOTOR_OUT_P 	   0.35f  //������������
#define THR_INTE_LIM   THR_INTE_LIM_SET/MOTOR_OUT_P 
#define THR_START      35.0f  //����������ٷֱ� %
#define MAX_THR_SET    85.0f  //������Űٷֱ� %

#define MOTOR_ESC_TYPE 1  //2����ˢ�����ɲ���ĵ����1����ˢ�������ɲ���ĵ��

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
    STABILIZE   = 0, //����
    LOC_HOLD    = 1, //������ͣ
    RETURN_HOME = 2, //����
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
}; //ң��������ͨ��

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
    G_X , //֮ǰ�ɰ������������ܴ� X��Y�ߵ���
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
    uint64_t time_us; // ϵͳ��΢����� 64bit
    uint32_t time_loop_fast_us; //ϵͳѭ���� ��ѭ��ʹ�õ�ʱ�� ��λus
    uint32_t time_loop_scheduler_sum_us; //ϵͳѭ���� ������ȱ�������ʹ�õ���ʱ�� ��λus
    uint32_t time_loop_scheduler_us[100]; //ϵͳѭ���� ������ȱ���ÿ������ʹ�õ�ʱ�� ��λus ����������100������
};  

struct T_GLOBAL_FLAG
{
    uint8_t power_ok;
    uint8_t init_ok;
    uint8_t sensor_imu_ok;
    uint8_t mems_temperature_ok; //���Դ��������ȿ��� ����ֱ����1
    
    //������״̬
    uint8_t gyro_ok;
    uint8_t acc_ok;
    uint8_t mag_ok;
    uint8_t baro_ok;
    uint8_t tof_ok; //�ɿ���Ӽ�����
    uint8_t of_ok; //����
    
    //���ƿ��� �Ƿ�ʹ��
    uint8_t gps_on;
    uint8_t of_flow_on;  //������Ϣ
    uint8_t of_tof_on;  //������������
    uint8_t tof_on;  //�ɿ���Ӽ�����
    
    //����ָ��
    uint8_t unlock_cmd;//����ָ�� ������ң����������վ����ȴ���
    uint8_t locking; //ң����ҡ�˴��ڽ���״̬
    
    //����״̬
    uint8_t unlockStatus; //����״̬
    uint8_t motor_preparation; //����������Ҫ׼��һС��ʱ��
    uint8_t motionless; 
    uint8_t taking_off; //���
    uint8_t flying; //���� wzy ��ɺͷ���״̬ע������ 
    uint8_t auto_take_off_land; //�Զ���ɺ��Զ������־λ
    uint8_t rc_loss_back_home; //��ʧң�����ź� ������־
};

struct T_GLOBAL_CONFIG
{
    uint8_t rcInMode;
    uint8_t flightMode;
};

struct T_GLOBAL_SAVE_DATA
{
	float acc_offset[VEC_XYZ]; //���ٶȼ���ƫ
	float gyro_offset[VEC_XYZ]; //��������ƫ
	float center_pos_cm[VEC_XYZ]; //������Դ�����λ��ƫ����	
	float mag_offset[VEC_XYZ]; //��������ƫ
	float mag_gain[VEC_XYZ];  //������У������
};  

extern struct T_GLOBAL_TIME globalSystemTime;
extern struct T_GLOBAL_FLAG globalFlag;
extern struct T_GLOBAL_CONFIG globalConfig;
extern struct T_GLOBAL_SAVE_DATA globalSaveData;

#endif /* GLOBAL_H_ */
