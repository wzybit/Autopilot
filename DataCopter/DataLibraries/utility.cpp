/*
 * utility.cpp
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */


/*
 * 系统相关头文件
 */
#include "stm32f4xx.h"

#include "stdio.h"
#include "math.h"
#include "string.h"

#include "global.h"
#include "utility.h"

uint64_t clock_gettime_ms()
{
    uint64_t time_ms=0.0;
    uint64_t system_tick;
    
    system_tick = globalSystemTime.system_tick_update_ms; //系统滴答定时器中断服务程序在 stm32f4xx_it.c中添加
    
    time_ms = system_tick;
    
    return time_ms;
}

uint64_t clock_gettime_us()
{
    uint64_t time_us=0.0;
    uint64_t system_tick;
    uint32_t load = 0;
    uint32_t val = 0;
    
    system_tick = clock_gettime_ms();
    
    load = SysTick->LOAD; // 这个SysTick->LOAD是24位的 所以用uint32_t就可以了
    val = SysTick->VAL;
    
    time_us = system_tick * US_PER_TICK + (uint64_t)( (float)((float)( load - val ) * US_PER_TICK) / (float)load );
    
    globalSystemTime.time_us = time_us;
    
    return time_us;
}


void delay_us ( uint64_t us )
{
    uint64_t now = (uint64_t)clock_gettime_us();
    while ( clock_gettime_us() - now < us );
}

void delay_ms ( uint32_t ms )
{
    while ( ms-- )
        delay_us ( 1000 );
}


