#ifndef RELAY_H
#define RELAY_H
#include "stm32f10x.h"

void Motor_Init(void);
void Motor_init(void);
void Spray_init(void);
void Motor_Left(u8 trun);
void Motor_Rigth(u8 trun);
void Motor_Counterclockwise(void);
void Motor_Clockwise(void);
void Rigth_Spray(u8 trun);
void Left_Tape(u8 trun);
void Scissors_servo(void);
void  Tape_servo(u8 trun);
void Run(void);
void Embrace_servo(u8 trun);
void Motor_dao( u8 turn );
void Pwm_Motor(u8 turn , u16 PWM);

#endif
