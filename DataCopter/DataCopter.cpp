/*
 *@File         : DataCopter.cpp
 *@Author       : wangbo
 *@Date         : Nov 4, 2017
 *@Copyright    : 2018 Beijing Institute of Technology. All right reserved.
 *@Warning      : This content is limited to internal use within the Complex Industrial Control Laboratory of Beijing Institute of Technology.
                  Prohibition of leakage and other commercial purposes.
 *@description  : Program entry and task list          
 */

#include "Copter.h"

#define SCHED_TASK(func) (void (*)())&Copter::func

const BIT_Scheduler::Task Copter::scheduler_tasks[] =
{
    //{ SCHED_TASK(静态函数名), 频率 单位Hz, 最大执行时间 单位us }, 
    //传感器数据处理
    { SCHED_TASK(update_MAG),      50,  200 },  //罗盘数据处理任务
//  { SCHED_TASK(update_AHRS),     500, 200 }, //APM的姿态解算暂时没有用，在快循环里用的匿名姿态结算
    { SCHED_TASK(update_height),   100, 200 }, //高度数据 处理 融合 （加速度计 气压计 激光 光流等）
//	{ SCHED_TASK(update_GPS),      10,  200 },    	
    { SCHED_TASK(read_radio),      100, 200 }, //遥控器数据读取

    //控制
    { SCHED_TASK(update_flight_mode),     100, 200 }, //飞行模式更新
    { SCHED_TASK(attitude_rate_control),  500, 200 }, //姿态内环控制 角速度环
    { SCHED_TASK(attitude_angle_control), 200, 200 }, //姿态外环控制 角度环
    { SCHED_TASK(height_control),         100, 200 }, //高度内外环控制 包括高度环和高度方向油门速率控制 这里油门控制为速率控制
    { SCHED_TASK(position_control),        50, 200 }, //水平方向位置控制 位置控制环输出给到角度环
    { SCHED_TASK(navigate_control),        50, 200 }, //导航控制 输出结果给到位置环
    
    //电机与舵机输出
    { SCHED_TASK(arm_motors_check),    100, 200 }, //电机解锁检查
    { SCHED_TASK(write_motors),        500, 200 }, //输出
    
    { SCHED_TASK(write_data_to_flash), 2,   900 }, //将数据写入flash保存 时间可能较长 需测量
    
    { SCHED_TASK(loop_100hz),       100, 200 },
    { SCHED_TASK(loop_10hz),        10,  200 },
    { SCHED_TASK(loop_1hz),         1,   200 },
    { SCHED_TASK(end_of_task),      1,   200 }
};

int main(int argc,char * const argv[])
{   
    globalFlag.init_ok = copter.setup();   
    copter.scheduler.init(&copter.scheduler_tasks[0], sizeof(copter.scheduler_tasks)/sizeof(copter.scheduler_tasks[0]));
    DEBUG_PRINTF("There are %d task to run!!!\n", sizeof(copter.scheduler_tasks)/sizeof(copter.scheduler_tasks[0]));

    while (1)
    {        
        copter.loop();
    }
}

void Copter::loop( void )
{
    //DEBUG_PRINTF("time_available_usec = %d \n", scheduler.time_available_usec());    
    wait_us(scheduler.time_available_usec());//将执行周期内剩余的时间浪费掉 保证执行周期准确
    
    uint64_t timer = (uint64_t)clock_gettime_us(); // loop start time in us
    
    loop_fast();
    globalSystemTime.time_loop_fast_us = (uint32_t)(clock_gettime_us() - timer); //计算下快循环使用的时间
    
    scheduler.tick();
        
    uint32_t loop_us = (uint32_t)(1000000 / scheduler.get_loop_rate_hz());
    uint32_t time_available = loop_us - (uint32_t)(clock_gettime_us() - timer);
    
    scheduler.run(time_available > loop_us ? 0u : time_available);
    globalSystemTime.time_loop_scheduler_sum_us = time_available - scheduler.time_available_usec();
}

void Copter::loop_fast()
{
    LED_1ms_Task();
    update_onboard_sensor_input();   //读取板载传感器数据 更新姿态数据
    update_flight_status();          //更新飞行状态
    update_external_sensor_switch(); //更新外部传感器开关
    
}

void Copter::loop_100hz()
{
    LED_10ms_Task(10);//灯光控制
}

void Copter::loop_10hz()
{

}

void Copter::loop_1hz()
{
    //LED test
//    static uint8_t led_cnt;
//    LED_R_ON;
//    LED_G_ON;
//    LED_B_ON;
//    if(led_cnt % 2 == 0)
//    {
//        LED_R_OFF;
//        LED_G_OFF;
//        LED_B_OFF;
//        led_cnt = 0;
//    }
//    led_cnt++;
}

void Copter::end_of_task()
{
	DEBUG_PRINTF("Hello end_of_task\n");
}


