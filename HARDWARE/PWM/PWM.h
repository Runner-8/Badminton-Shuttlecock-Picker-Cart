#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

// PWM周期最大值，ARR=39，对应100%
#define PWM_MAX_CCR    39U

// 通道枚举，增强可读性
typedef enum
{
    FAN_CH1 = 1,  // TIM2_CH1 风扇1
    FAN_CH2 = 2   // TIM2_CH2 风扇2
}FanChannel_t;

void Fan_Init(void);
void Fan_SetSpeed(FanChannel_t ch, uint8_t percent);


#endif
