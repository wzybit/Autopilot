/*
 * utility.h
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include "stdint.h"

uint64_t clock_gettime_ms(void);
uint64_t clock_gettime_us(void);

#ifdef  __cplusplus  
extern "C" {  
#endif     
void delay_us ( uint64_t us );
void delay_ms ( uint32_t ms );
#ifdef  __cplusplus  
}  
#endif 
//uint32_t GetSysTime_us ( void );

#endif /* UTILITY_H_ */
