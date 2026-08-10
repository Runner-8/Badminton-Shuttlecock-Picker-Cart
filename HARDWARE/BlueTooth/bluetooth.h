#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include "stm32f10x.h"
#include <stdint.h>

/************************************************
* 帧协议定义
************************************************/
#define FRAME_HEADER        0xAA
#define FRAME_TAIL          0x55
#define BT_RX_BUF_SIZE         64
#define RX_TIMEOUT_MS       100

/************************************************
* 命令枚举
************************************************/
typedef enum {
    CMD_NONE = 0,
    CMD_FORWARD = 1,         // 前进
    CMD_BACKWARD = 2,        // 后退
    CMD_LEFT = 3,            // 左转
    CMD_RIGHT = 4,           // 右转
    CMD_STOP = 5,            // 停止
    CMD_CRUISE_TOGGLE = 6,   // 自动巡航开关（带1字节：0关1开）
    CMD_SET_THRESHOLD = 7,   // 设置阈值（带4字节float数据）
    CMD_LED = 8, 		     // 紫外线开关
    CMD_AUTO_RUN = 9, 	     // 自动移动
	CMD_SEND_THRESHOLD = 10, // 阈值上报
	CMD_SEND_FLAG = 11		 // 超重标志位上报
} Command_t;

#define CAR_STOP             0
#define CAR_FORWARD          1
#define CAR_BACKWARD         2
#define CAR_LEFT             3
#define CAR_RIGHT            4


//自动返航状态枚举
typedef enum {
    RETURN_IDLE = 0,      // 空闲，等待超重触发
    RETURN_BACKING,       // 正在返航（旋转+直行）
    RETURN_ARRIVED        // 已到达起始点，停车
} ReturnState_t;

/************************************************
* 状态变量（外部可访问）
************************************************/
volatile extern float g_threshold;           		 // 当前阈值
volatile extern uint8_t g_cruise_enabled;    		 // 巡航开关
volatile extern uint8_t g_led_enabled;				 // 紫外线开关
volatile extern uint8_t g_auto_run;					 // 自动移动
volatile extern uint8_t g_auto_back;				 // 自动返航
volatile extern uint8_t g_auto_arrived;				 // 返航完成
volatile extern uint8_t g_car_state;         		 // 小车状态
volatile extern uint8_t turn_direction;				 // 转向标志 0-无 1-右转 2-左转
extern uint8_t g_rx_buffer[BT_RX_BUF_SIZE];  		 // 串口接收缓冲区
extern uint8_t g_rx_index;
extern volatile ReturnState_t g_return_state;		 // 自动返航状态枚举变量

/************************************************
* 函数声明
************************************************/
void Usart3_Init(uint32_t baud);
void BT_SendString(char *str);
void BT_SendFrame(uint8_t cmd, uint8_t *data, uint8_t data_len);
void BT_ProcessReceivedData(void); 
void Weight_Monitor(void);

#endif
