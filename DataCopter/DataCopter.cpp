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
    //{ SCHED_TASK(��̬������), Ƶ�� ��λHz, ���ִ��ʱ�� ��λus }, 
    //���������ݴ���
    { SCHED_TASK(update_MAG),      50,  200 },  //�������ݴ�������
//  { SCHED_TASK(update_AHRS),     500, 200 }, //APM����̬������ʱû���ã��ڿ�ѭ�����õ�������̬����
    { SCHED_TASK(update_height),   100, 200 }, //�߶����� ���� �ں� �����ٶȼ� ��ѹ�� ���� �����ȣ�
//	{ SCHED_TASK(update_GPS),      10,  200 },    	
    { SCHED_TASK(read_radio),      100, 200 }, //ң�������ݶ�ȡ

    //����
    { SCHED_TASK(update_flight_mode),     100, 200 }, //����ģʽ����
    { SCHED_TASK(attitude_rate_control),  500, 200 }, //��̬�ڻ����� ���ٶȻ�
    { SCHED_TASK(attitude_angle_control), 200, 200 }, //��̬�⻷���� �ǶȻ�
    { SCHED_TASK(height_control),         100, 200 }, //�߶����⻷���� �����߶Ȼ��͸߶ȷ����������ʿ��� �������ſ���Ϊ���ʿ���
    { SCHED_TASK(position_control),        50, 200 }, //ˮƽ����λ�ÿ��� λ�ÿ��ƻ���������ǶȻ�
    { SCHED_TASK(navigate_control),        50, 200 }, //�������� ����������λ�û�
    
    //����������
    { SCHED_TASK(arm_motors_check),    100, 200 }, //����������
    { SCHED_TASK(write_motors),        500, 200 }, //���
    
    { SCHED_TASK(write_data_to_flash), 2,   900 }, //������д��flash���� ʱ����ܽϳ� �����
    
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
    wait_us(scheduler.time_available_usec());//��ִ��������ʣ���ʱ���˷ѵ� ��ִ֤������׼ȷ
    
    uint64_t timer = (uint64_t)clock_gettime_us(); // loop start time in us
    
    loop_fast();
    globalSystemTime.time_loop_fast_us = (uint32_t)(clock_gettime_us() - timer); //�����¿�ѭ��ʹ�õ�ʱ��
    
    scheduler.tick();
        
    uint32_t loop_us = (uint32_t)(1000000 / scheduler.get_loop_rate_hz());
    uint32_t time_available = loop_us - (uint32_t)(clock_gettime_us() - timer);
    
    scheduler.run(time_available > loop_us ? 0u : time_available);
    globalSystemTime.time_loop_scheduler_sum_us = time_available - scheduler.time_available_usec();
}

void Copter::loop_fast()
{
    LED_1ms_Task();
    update_onboard_sensor_input();   //��ȡ���ش��������� ������̬����
    update_flight_status();          //���·���״̬
    update_external_sensor_switch(); //�����ⲿ����������
    
}

void Copter::loop_100hz()
{
    LED_10ms_Task(10);//�ƹ����
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


