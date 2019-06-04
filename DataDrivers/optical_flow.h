
#ifndef OPTICAL_FLOW_H
#define	OPTICAL_FLOW_H

#include <stdint.h>

extern uint8_t 	OF_STATE,OF_QUALITY;

//融合后的光流信息，具体意义见光流模块手册
extern int16_t	OF_DX2,OF_DY2,OF_DX2FIX,OF_DY2FIX;

//原始高度信息和融合后高度信息
extern uint16_t	OF_ALT,OF_ALT2;

#endif //OPTICAL_FLOW_H
