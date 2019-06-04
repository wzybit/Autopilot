
#ifndef DRIVER_PWM_OUT_H
#define	DRIVER_PWM_OUT_H

#include "stm32f4xx.h"
#include "global.h"

//通过改变TIMx->CCRx的值来改变占空比，从而控制电调
#define PWM1 TIM3->CCR2
#define PWM2 TIM3->CCR1
#define PWM3 TIM3->CCR3
#define PWM4 TIM3->CCR4
#define PWM5 TIM4->CCR1
#define PWM6 TIM4->CCR2

#ifdef  __cplusplus  
extern "C" {  
#endif    
void PWM_Out_Init(uint16_t hz);
void write_pwm_all(int16_t motor_pwm[MOTORSNUM]);

#ifdef  __cplusplus  
}  
#endif 

#endif //DRIVER_PWM_OUT_H
