
#ifndef DRIVER_PWM_IN_H
#define	DRIVER_PWM_IN_H

#include "stm32f4xx.h"

#define PPM_CHNUM 8

#ifdef  __cplusplus  
extern "C" {  
#endif    
void PWM_In_Init(u8 pwmInMode);

#ifdef  __cplusplus  
}  
#endif 

extern u16 Rc_Pwm_In[8];
extern u16 Rc_Ppm_In[PPM_CHNUM];

#endif //DRIVER_PWM_IN_H
