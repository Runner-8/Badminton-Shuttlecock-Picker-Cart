#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

// 初始化舵机（PWM和初始角度）
void Servo_Init(void);

// 设置水平角度（0~270°）
void Servo_SetAngle_H(float angle_deg);

// 设置垂直角度（0~180°）
void Servo_SetAngle_V(float angle_deg);

// 当前角度（供外部读取或修改）
extern float current_angle_h;
extern float current_angle_v;

#endif
