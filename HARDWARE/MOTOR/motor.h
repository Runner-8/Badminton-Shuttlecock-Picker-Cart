#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"
#include "stm32f10x.h"                  // Device header
#include "motor.h"


void LED_Init(void);
void ir_gpio_init(void);
void Motor_Init(void);          // 初始化GPIO
void Motor_Stop(void);          // 全停
void Motor_Forward(void);       // 前进
void Motor_Backward(void);      // 后退
void Motor_Left(void);          // 左转（原地左旋）
void Motor_Right(void);         // 右转（原地右旋）

#endif

