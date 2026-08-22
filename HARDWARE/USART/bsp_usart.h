#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stm32f10x.h"
#include <stdint.h>
#include <string.h>

#define RX_BUF_SIZE     128
#define MAX_OBJECTS     8

// 目标数据结构
typedef struct
{
    char label[8];
    float cx;
    float cy;
    int w;
    int h;
}Object_t;

// 串口初始化（USART1）
void Usart1_Init(uint32_t baud);

// 获取接收标志（1：收到完整帧）
uint8_t Uart_GetFrameFlag(void);

// 清除接收标志
void Uart_ClearFrameFlag(void);

// 获取帧数据指针（只读）
const char* Uart_GetFrameData(void);

// 解析帧数据，返回目标个数
int parse_frame(const char *data, Object_t *objs, int max_objs);

#endif
