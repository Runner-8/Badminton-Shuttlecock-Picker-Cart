#include "bsp_usart.h"
#include <stdio.h>
#include "bluetooth.h"
#include <string.h>
#include <stdarg.h>   // 新增，用于 va_list

// 内部接收状态
typedef struct {
    uint8_t buf[RX_BUF_SIZE];
    volatile uint8_t len;
    volatile uint8_t state;   // 0:等待头，1:接收数据
    volatile uint8_t flag;    // 1:收到完整帧
} UartRx_t;

static UartRx_t g_rx = {0};

// ===================== 串口初始化 =====================
void Usart1_Init(uint32_t baud)
{
    GPIO_InitTypeDef gpioInitStruct;
    USART_InitTypeDef usartInitStruct;
    NVIC_InitTypeDef nvicInitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // PA9 TX
    gpioInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioInitStruct.GPIO_Pin = GPIO_Pin_9;
    gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpioInitStruct);

    // PA10 RX
    gpioInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpioInitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpioInitStruct);

    usartInitStruct.USART_BaudRate = baud;
    usartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usartInitStruct.USART_Parity = USART_Parity_No;
    usartInitStruct.USART_StopBits = USART_StopBits_1;
    usartInitStruct.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &usartInitStruct);

    USART_Cmd(USART1, ENABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    nvicInitStruct.NVIC_IRQChannel = USART1_IRQn;
    nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
    nvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    nvicInitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvicInitStruct);
}

// ===================== 串口中断 =====================
void USART1_IRQHandler(void)
{
    uint8_t res;   // 声明放在函数开头

    if (USART_GetITStatus(USART1, USART_IT_RXNE) == RESET)
        return;

    res = USART_ReceiveData(USART1);

    switch (g_rx.state) {
        case 0:
            if (res == '#') {
                g_rx.len = 0;
                g_rx.state = 1;
                memset(g_rx.buf, 0, RX_BUF_SIZE);
            }
            break;
        case 1:
            if (res == '*') {
                g_rx.buf[g_rx.len] = '\0';
                g_rx.flag = 1;
                g_rx.state = 0;
                g_rx.len = 0;
            } else if (res == '#') {
                g_rx.len = 0;
                g_rx.state = 1;
                memset(g_rx.buf, 0, RX_BUF_SIZE);
            } else if (res != '\n' && res != '\r') {
                if (g_rx.len < RX_BUF_SIZE - 1) {
                    g_rx.buf[g_rx.len++] = res;
                } else {
                    g_rx.state = 0;
                    g_rx.len = 0;
                }
            }
            break;
    }
}

// ===================== 对外接口 =====================
uint8_t Uart_GetFrameFlag(void)
{
    return g_rx.flag;
}

void Uart_ClearFrameFlag(void)
{
    g_rx.flag = 0;
}

const char* Uart_GetFrameData(void)
{
    return (const char*)g_rx.buf;
}

// ===================== 解析函数 =====================
int parse_frame(
const char *data,
Object_t *objs,
int max_objs)
{
    char work_buf[RX_BUF_SIZE];
    char *token;
    int count=0;
    if(data==NULL)
        return 0;
    if(strcmp(data,"NONE")==0)
        return 0;
    strncpy(
    work_buf,
    data,
    RX_BUF_SIZE-1);
    work_buf[RX_BUF_SIZE-1]='\0';
    token=strtok(work_buf,";");
    while(token!=NULL &&
          count<max_objs)
    {
        if(sscanf(token,
        "%7[^,],%f,%f,%d,%d,%f",
        objs[count].label,
        &objs[count].cx,
        &objs[count].cy,
        &objs[count].w,
        &objs[count].h,
        &objs[count].score)==6)
        {
            count++;
        }
        token=strtok(NULL,";");
    }
    return count;

}

void UsartPrintf(USART_TypeDef *USARTx, char *fmt, ...)
{
    char buf[256];
    va_list ap;
    char *p;                     // 声明放在开头

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    p = buf;                     // 现在可以赋值了
    while (*p) {
        USART_SendData(USARTx, *p++);
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
    }
}

int fputc(int ch, FILE *f)
{
	while(!(USART1->SR & USART_FLAG_TXE));
	
	USART1->DR = (uint8_t)ch;

	return ch;
}
