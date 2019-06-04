
#ifndef AK8975_H
#define	AK8975_H

#include "stm32f4xx.h"

#define AK8975_ADDRESS         0x0c	// 0x18

#define AK8975_WIA     0x00
#define AK8975_HXL     0x03
#define AK8975_HXH     0x04
#define AK8975_HYL     0x05
#define AK8975_HYH     0x06
#define AK8975_HZL     0x07
#define AK8975_HZH     0x08
#define AK8975_CNTL    0x0A

#ifdef  __cplusplus  
extern "C" {  
#endif 
u8 AK8975_Init(void);
void AK8975_Read(void);
void Mag_Get(void);
    
#ifdef  __cplusplus  
}  
#endif 
extern s16 mag_val[3];
#endif //AK8975_H
