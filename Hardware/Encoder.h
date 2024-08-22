#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f1xx_hal.h"
extern int Encoder_Left, Encoder_Right;

void Encoder_Init(void);
int GetEncoderVal(TIM_HandleTypeDef *TIMx);

#endif // __ENCODER_H