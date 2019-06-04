
#ifndef DRIVER_USART_H
#define DRIVER_USART_H

#include "stm32f4xx.h"

extern uint16_t rc_sbus_in[16];
#ifdef  __cplusplus  
extern "C" {  
#endif 
void USART1_Init(u32 baud);
void USART1_Send(unsigned char *DataToSend ,u8 data_num);

void USART2_Init(u32 baud);
void USART2_Send(unsigned char *DataToSend ,u8 data_num);
    
void USART3_Init(u32 baud);
void USART3_Send(unsigned char *DataToSend ,u8 data_num);
    
#ifdef  __cplusplus  
}  
#endif  

#endif //DRIVER_USART_H
