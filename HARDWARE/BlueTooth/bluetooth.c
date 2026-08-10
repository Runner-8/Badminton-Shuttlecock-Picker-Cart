#include "bluetooth.h"
#include "stdio.h"
#include "string.h"
#include "HX711.h"

/************************************************
* 全局变量
************************************************/
volatile float g_threshold = 30.0f;
volatile uint8_t g_cruise_enabled = 0;
volatile uint8_t g_led_enabled = 0;
volatile uint8_t g_auto_run = 0;
volatile uint8_t g_auto_back = 0;
volatile uint8_t g_auto_arrived = 0;
volatile uint8_t g_car_state = CAR_STOP;
uint8_t g_rx_buffer[BT_RX_BUF_SIZE];
uint8_t g_rx_index = 0;
uint8_t g_frame_ready = 0;  // 帧接收完成标志
extern volatile uint32_t g_rx_timeout; 

float current_weight = 0.0f;
static uint32_t last_weight_check_time = 0;
static uint32_t weight_below_start_time = 0;
static uint8_t is_auto_moving = 0;
static uint32_t auto_move_start_time = 0;

volatile ReturnState_t g_return_state = RETURN_IDLE;
volatile uint8_t turn_direction;

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
void BT_SendFrame(uint8_t cmd, uint8_t *data, uint8_t data_len)
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
			g_rx_timeout = 0;
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
		case CMD_CRUISE_TOGGLE:
            if (data_len == 1) {
                g_cruise_enabled = data[0] ? 1 : 0;
			}
            break;
        case CMD_SET_THRESHOLD:
            if (data_len == 4) {
                // 小端模式解析浮点数
                float val = *((float*)data);
                g_threshold = val;
            }
            break;
        case CMD_LED:
            // 紫外线开关
			if (data_len == 1) {
                g_led_enabled = data[0] ? 1 : 0;
            }
            break;
		case CMD_AUTO_RUN:
            // 紫外线开关
			if (data_len == 1) {
                g_auto_run = data[0] ? 1 : 0;
            }
            break;
        default:
            //BT_SendString("CMD:Unknown\r\n");
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
    
    if (g_rx_index > 0 && g_rx_timeout >= RX_TIMEOUT_MS)
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
        g_rx_timeout = 0;   // 复位超时计数器
	}
}

/************************************************
*
* 重量检测
* 每5秒获取一下重量数， 每15秒与阈值进行比较.
*
************************************************/
void Weight_Monitor(void)
{
    uint32_t tick = get_tick();
    static uint8_t last_alert_sent = 0;

    if (tick - last_weight_check_time >= 3000) {
        last_weight_check_time = tick;
        current_weight = Get_Weight();
        BT_SendFrame(CMD_SEND_THRESHOLD, (uint8_t *)&current_weight, 4);

        // ===== 超重检测（最高优先级） =====
        if (current_weight >= g_threshold) {
            // 1. 如果正在自动移动，立即中断并停车
            if (is_auto_moving) {
                is_auto_moving = 0;
                g_car_state = CAR_STOP;
            }
            weight_below_start_time = 0;

            // 2. 发送超重标志（仅一次）
            if (!last_alert_sent) {
                BT_SendFrame(CMD_SEND_FLAG, NULL, 0);
                last_alert_sent = 1;
            }

			// ★★★ 触发返航的条件 ★★★
            // 只要当前状态是“空闲”，立即触发返航
            if (g_return_state == RETURN_IDLE) {
                g_return_state = RETURN_BACKING;  // 切换为返航中
            }
        } 
        else { // 重量低于阈值
            // 清除超重标志，以便下次超重再发送
            last_alert_sent = 0;

            // 如果当前正在返航或已到达，则立即停止并重置状态
            if (g_return_state != RETURN_IDLE) {
                g_return_state = RETURN_IDLE;
                g_car_state = CAR_STOP;
                turn_direction = 0;   // 清除转向保持
                // 可选：发送状态信息
                // BT_SendString("Return cancelled\r\n");
            }

            // ===== 自动移动功能（仅当 g_auto_run 开启） =====
            if (g_auto_run) {
                if (weight_below_start_time == 0) {
                    weight_below_start_time = tick;
                } else if (tick - weight_below_start_time >= 15000 && !is_auto_moving) {
                    is_auto_moving = 1;
                    auto_move_start_time = tick;
                    g_car_state = CAR_FORWARD;
                }
            } else {
                weight_below_start_time = 0;
                if (is_auto_moving) {
                    is_auto_moving = 0;
                    g_car_state = CAR_STOP;
                }
            }
        }
    }

    // ===== 自动移动执行（前进1秒后停止） =====
    if (is_auto_moving) {
        if (tick - auto_move_start_time >= 1000) {
            is_auto_moving = 0;
            g_car_state = CAR_STOP;
            weight_below_start_time = tick;
        }
    }
}
