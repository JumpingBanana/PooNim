#ifndef __ONEIROI_MOTOR_H
#define __ONEIROI_MOTOR_H

#include "stdint.h"

typedef struct 
{
	volatile int EncoderCount;
	int Direction;
	uint32_t ADC_raw;
	volatile int flag_update;
}MOTOR_HandlerTypeDef;

#endif /* __ONEIROI_MOTOR_H */

/************************  END OF FILE  ************************/
