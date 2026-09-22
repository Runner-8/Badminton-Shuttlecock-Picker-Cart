#include "bluetooth.h"
#include "stdio.h"
#include "string.h"
#include "HX711.h"

/************************************************
* 全局变量
************************************************/
volatile uint8_t g_suction = 0;
volatile uint8_t g_auto_arrived = 0;
volatile uint8_t g_mode = 0;
volatile uint8_t g_car_state = CAR_STOP;
uint8_t g_rx_buffer[BT_RX_BUF_SIZE];
uint8_t g_rx_index = 0;
//uint8_t g_frame_ready = 0;  // 帧接收完成标志
volatile ReturnState_t g_return_state = RETURN_IDLE;
volatile uint8_t turn_direction;
volatile uint32_t g_rx_timeout;

/************************************************
*
* 串口初始化
*
************************************************/
void Usart3_Init(uint32_t baud)
{
    GPIO_InitTypeDef gpioInitStruct;
    USART_InitTypeDef usartInitStruct;
    NVIC_InitTypeDef nvicInitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // PB10 TX
    gpioInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioInitStruct.GPIO_Pin = GPIO_Pin_10;
    gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioInitStruct);

    // PB11 RX
    gpioInitStruct.GPIO_Mode = GPIO_Mode_IPU;
    gpioInitStruct.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &gpioInitStruct);

    usartInitStruct.USART_BaudRate = baud;
    usartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usartInitStruct.USART_Parity = USART_Parity_No;
    usartInitStruct.USART_StopBits = USART_StopBits_1;
    usartInitStruct.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART3, &usartInitStruct);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
	
    nvicInitStruct.NVIC_IRQChannel = USART3_IRQn;
    nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
    nvicInitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    nvicInitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvicInitStruct);
}

/************************************************
*
* 发送数据
*
************************************************/
void BT_SendData(uint8_t *data, uint16_t len)
{
    while (len--) {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
        USART_SendData(USART3, *data++);
    }
}

/************************************************
*
* 串口发送字符串
*
************************************************/
void BT_SendString(char *str)
{
    while (*str) {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
        USART_SendData(USART3, *str++);
    }
}

/************************************************
*
* 发送封装好的数据帧
*
*帧头+长度+命令+数据+校验+帧尾
*
*长度=命令+数据+校验
*
************************************************/
void BT_SendFrame(uint8_t cmd, volatile uint8_t *data, uint8_t data_len)
{
    uint8_t frame[32];
    uint8_t idx = 0;
    uint8_t i;
    uint8_t checksum = 0;
	uint8_t len = 0;

    // 帧头
    frame[idx++] = FRAME_HEADER;
    // 长度 = 命令1字节 + 数据n字节 + 校验1字节
    len = 1 + data_len + 1;
    frame[idx++] = len;
    // 命令
    frame[idx++] = cmd;
    // 数据
    for (i = 0; i < data_len; i++) {
        frame[idx++] = data[i];
    }
    // 校验（从len到数据末尾的异或）
    for (i = 1; i < idx; i++) {
        checksum ^= frame[i];
    }
    frame[idx++] = checksum;
    // 帧尾
    frame[idx++] = FRAME_TAIL;

    BT_SendData(frame, idx);
}

/************************************************
*
* 串口中断：只负责收数据，不做任何解析
*
************************************************/
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART3);
        // 简单的循环缓冲区，防止溢出
        if (g_rx_index < BT_RX_BUF_SIZE) {
            g_rx_buffer[g_rx_index++] = data;
			g_rx_timeout = get_tick();
        }
        
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}

/************************************************
*
* 帧解析器：在主循环中调用
* 返回 1 表示成功解析一帧，0 表示未完成
*
************************************************/
static uint8_t ParseFrame(uint8_t *buf, uint8_t len, uint8_t *out_cmd, uint8_t *out_data, uint8_t *out_data_len)
{
	
    uint8_t i = 0;
	uint8_t j = 0;
    uint8_t frame_len;
    uint8_t checksum = 0;

    // 找帧头
    while (i < len && buf[i] != FRAME_HEADER) i++;
    if (i >= len) return 0;  // 没找到帧头
    
    // 检查剩余长度是否够一个最小帧（AA + len + cmd + check + 55 = 5字节）
    if (len - i < 5) return 0;
    
    // 读取长度
    frame_len = buf[i + 1];
    if (frame_len < 2 || frame_len > 20) return 0;  // 长度不合理
    
    // 检查整个帧是否都收到了
    if (len - i < (uint8_t)(frame_len + 2)) return 0;  // +2 是帧头和帧尾
    
    // 校验帧尾
    if (buf[i + 2 + frame_len] != FRAME_TAIL) return 0;
    
    // 校验和
    for (j = 1; j < frame_len + 1; j++) {  // 从长度到校验前一字节
        checksum ^= buf[i + j];
    }
    if (checksum != buf[i + 1+ frame_len]) return 0;  // 校验失败
    
    // 取出命令和数据
    *out_cmd = buf[i + 2];
    *out_data_len = frame_len - 2;  // 总长度 - 命令 - 校验
    if (*out_data_len > 0) {
        memcpy(out_data, &buf[i + 3], *out_data_len);
    }

    return 1;
}

/************************************************
*
* 命令执行器
*
************************************************/
static void ExecuteCommand(uint8_t cmd, uint8_t *data, uint8_t data_len)
{
    switch (cmd) {
        case CMD_FORWARD:
            g_car_state = CAR_FORWARD;
            break;
        case CMD_BACKWARD:
            g_car_state = CAR_BACKWARD;
            break;
        case CMD_LEFT:
            g_car_state = CAR_LEFT;
            break;
        case CMD_RIGHT:
            g_car_state = CAR_RIGHT;
            break;
        case CMD_STOP:
            g_car_state = CAR_STOP;
            break;
        case CMD_SUCK:
            if (data_len == 1) {
                g_suction = data[0] ? 1 : 0;
			}
            break;
		case CMD_MODESWITCH:
            if (data_len == 1) {
                g_mode = data[0] ? 1 : 0;
			}
            break;
        default:
            BT_SendString("CMD:Unknown\r\n");
            break;
    }
}

/************************************************
*
* 主循环调用的处理函数
*
************************************************/
void BT_ProcessReceivedData(void)
{
    uint8_t cmd;
    uint8_t data[16];
    uint8_t data_len;
    
    if (g_rx_index > 0 && get_tick() - g_rx_timeout >= RX_TIMEOUT_MS)
	{
		// 尝试解析帧
		if (ParseFrame(g_rx_buffer, g_rx_index, &cmd, data, &data_len)) {
			// 解析成功，执行命令
			ExecuteCommand(cmd, data, data_len);
			// 清空缓冲区（准备接收下一帧）
			g_rx_index = 0;
			memset(g_rx_buffer, 0, BT_RX_BUF_SIZE);
		} else {
			//BT_SendString("ERR: Frame parse fail\r\n");
		}
		g_rx_index = 0;
        memset((uint8_t*)g_rx_buffer, 0, BT_RX_BUF_SIZE);
	}
}
