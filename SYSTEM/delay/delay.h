#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  
	 
void delay_init(void);
void delay_ms(uint32_t count);
void delay_us(u32 nus);
void SysTick_Init(void);
void SysTick_Increment(void);
uint32_t get_tick(void);

#endif





























