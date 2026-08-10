#include "stm32f10x.h"
#include "bsp_usart.h"
#include "PWM.h"
#include "bluetooth.h"
#include "stdlib.h"
#include "motor.h"
#include "HX711.h"
#include "delay.h"
#include <math.h>

/************************************************
* 图像中心
************************************************/
#define IMAGE_CENTER_X       112
#define IMAGE_CENTER_Y       112

/************************************************
* 红外补偿
*
*先关闭
*后期打墙标定
*
************************************************/
#define LASER_OFFSET_X       0
#define LASER_OFFSET_Y       0

/************************************************
* 
*目标点
*
*使用YOLO框中心
*
************************************************/
#define TARGET_OFFSET_X      0
#define TARGET_OFFSET_Y      -2

/************************************************
* 
*P控制参数
*
************************************************/
#define KP_H                 0.020f
#define KP_V                 0.018f

/************************************************
* 
*大范围死区
*
************************************************/
#define DEAD_X               6
#define DEAD_Y               8

/************************************************
* 
*精准锁定死区
*
************************************************/
#define AIM_STOP_X           3
#define AIM_STOP_Y           5

/************************************************
* 
*最大角度变化
*
************************************************/
#define MAX_STEP_H           0.25f
#define MAX_STEP_V           0.15f

/************************************************
*
* 控制滤波
*
************************************************/
#define FILTER_H             0.80f
#define FILTER_V             0.85f

/************************************************
*
* 云台范围
*
************************************************/
#define H_MIN                0
#define H_MAX                270
#define V_MIN                100
#define V_MAX                150

/************************************************
*
* 搜索参数
*
************************************************/
#define SEARCH_H_START       40.0f
#define SEARCH_H_END         270.0f
#define SEARCH_H_STEP        5.0f
#define SEARCH_H_DELAY       160 //40000
#define SEARCH_V_CENTER      125.0f
#define SEARCH_V_OFFSET      1.0f
#define SEARCH_V_DELAY       30//800


/************************************************
*
* 锁定参数
*
************************************************/
#define LOCK_FRAME_NUM       8

uint8_t lock_count = 0;

/************************************************
*
* 状态
*
************************************************/
#define SEARCH_MODE          0
#define TRACK_MODE           1

uint8_t system_state = SEARCH_MODE;


/************************************************
*
* 搜索变量
*
************************************************/
float search_h = SEARCH_H_START;
float search_v = SEARCH_V_CENTER;
uint8_t search_step = 0;
uint32_t timer_h = 0;
uint32_t timer_v = 0;
// 定义静态变量记录上次动作的时间
static uint32_t last_v_time = 0;
static uint32_t last_h_time = 0;


/************************************************
*
* 红外引脚定义
*
************************************************/
#define IR_RIGHT 		GPIO_Pin_1
#define IR_LEFT  		GPIO_Pin_0
#define IR_CENTER 		GPIO_Pin_3

//ON_TIME / (ON_TIME + OFF_TIME)

// 直行动作（前进/后退）的间歇参数
#define MOVE_ON_TIME    50   // 前进/后退 持续运动时间 (ms)
#define MOVE_OFF_TIME   50   // 前进/后退 停止时间 (ms)

// 转向动作（左转/右转）的间歇参数（转得慢一点，更稳
#define TURN_ON_TIME    50   // 转向持续运动时间 (ms)
#define TURN_OFF_TIME   150  // 转向停止时间 (ms)

static uint8_t actual_state = CAR_STOP;   // 当前实际输出状态
static uint32_t last_toggle_time = 0;
static uint8_t speed_step = 0;            // 0: 运动阶段, 1: 停止阶段
static uint32_t turn_start_time = 0;

/************************************************
*
* 目标锁定
*
************************************************/
void Lock_LED(uint8_t flag)
{
    if (flag) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
    } else {
        GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
    }
}

/************************************************
*
* 紫外线控制
*
************************************************/
void led_switch(void)
{
	if (g_led_enabled) {
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
    } else {
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
    }
}

/************************************************
*
* 自动巡检
*
************************************************/
void Search_Control(void)
{
	uint32_t now = get_tick();
	
    switch (search_step) {
        case 0:		
			if (now - last_v_time < SEARCH_V_DELAY) 
				return;
			last_v_time = now;

            search_v = SEARCH_V_CENTER;
            search_step = 1;
            break;

        case 1:
			if (now - last_v_time < SEARCH_V_DELAY) 
				return;
			last_v_time = now;
		
            search_v = SEARCH_V_CENTER - SEARCH_V_OFFSET;
            search_step = 2;
            break;

        case 2:
			if (now - last_v_time < SEARCH_V_DELAY) 
				return;
			last_v_time = now;
		
            search_v = SEARCH_V_CENTER;
            search_step = 3;
            break;

        case 3:
			if (now - last_v_time < SEARCH_V_DELAY) 
				return;
			last_v_time = now;
		
            search_v = SEARCH_V_CENTER + SEARCH_V_OFFSET;
            search_step = 4;
            break;

        case 4:
			if (now - last_h_time < SEARCH_V_DELAY) 
				return;
			last_h_time = now;
		
            search_h += SEARCH_H_STEP;
            if (search_h >= SEARCH_H_END) {
                search_h = SEARCH_H_START;
            }
            search_step = 0;
            break;
    }

    Servo_SetAngle_H(search_h);
    Servo_SetAngle_V(search_v);
}

/************************************************
* 
*视觉跟踪
*
************************************************/
void Track_Control(Object_t *obj)
{
    float err_x;
    float err_y;
    float delta_h;
    float delta_v;
    static float last_h = 0;
    static float last_v = 0;
    float target_x;
    float target_y;

    target_x = obj->cx + TARGET_OFFSET_X;
    target_y = obj->cy + TARGET_OFFSET_Y;

    err_x = target_x - (IMAGE_CENTER_X + LASER_OFFSET_X);
    err_y = target_y - (IMAGE_CENTER_Y + LASER_OFFSET_Y);

    if (fabs(err_x) < DEAD_X)
        err_x = 0;
    if (fabs(err_y) < DEAD_Y)
        err_y = 0;

    if (fabs(err_x) < AIM_STOP_X)
        err_x = 0;
    if (fabs(err_y) < AIM_STOP_Y)
        err_y = 0;

    delta_h = -err_x * KP_H;
    delta_v = err_y * KP_V;

    delta_h = FILTER_H * delta_h + (1 - FILTER_H) * last_h;
    delta_v = FILTER_V * delta_v + (1 - FILTER_V) * last_v;

    last_h = delta_h;
    last_v = delta_v;

    if (delta_h > MAX_STEP_H)
        delta_h = MAX_STEP_H;
    if (delta_h < -MAX_STEP_H)
        delta_h = -MAX_STEP_H;
    if (delta_v > MAX_STEP_V)
        delta_v = MAX_STEP_V;
    if (delta_v < -MAX_STEP_V)
        delta_v = -MAX_STEP_V;

    current_angle_h += delta_h;
    current_angle_v += delta_v;

    if (current_angle_h < H_MIN)
        current_angle_h = H_MIN;
    if (current_angle_h > H_MAX)
        current_angle_h = H_MAX;
    if (current_angle_v < V_MIN)
        current_angle_v = V_MIN;
    if (current_angle_v > V_MAX)
        current_angle_v = V_MAX;

    Servo_SetAngle_H(current_angle_h);
    Servo_SetAngle_V(current_angle_v);
}

/************************************************
* 
*稳定判断
*
************************************************/
uint8_t Check_Lock(Object_t *obj)
{
    int dx;
    int dy;

    dx = abs(obj->cx - IMAGE_CENTER_X);
    dy = abs(obj->cy - IMAGE_CENTER_Y);

    if (dx < DEAD_X && dy < DEAD_Y) {
        lock_count++;
    } else {
        lock_count = 0;
    }

    if (lock_count >= LOCK_FRAME_NUM)
        return 1;
    return 0;
}

/************************************************
*
* 小车控制
*
************************************************/
void Car_Control(uint8_t state)
{
    switch (state) {
        case CAR_FORWARD:
            Motor_Forward();
            break;
        case CAR_BACKWARD:
            Motor_Backward();
            break;
        case CAR_LEFT:
            Motor_Left();
            break;
        case CAR_RIGHT:
            Motor_Right();
            break;
        case CAR_STOP:
			Motor_Stop();
        default:
            Motor_Stop();
            break;
    }
}

/************************************************
*
* 自动巡检
*
************************************************/
void auto_cruise(Object_t obj)
{
	if (g_cruise_enabled) {
		if (Uart_GetFrameFlag()) {
			Uart_ClearFrameFlag();

			if (parse_frame(Uart_GetFrameData(), &obj, 1)) {
				system_state = TRACK_MODE;
				Track_Control(&obj);

				if (Check_Lock(&obj)) {
					Lock_LED(1);
				}
			} else {
				system_state = SEARCH_MODE;
				lock_count = 0;
				Lock_LED(0);
			}
	}

		if (system_state == SEARCH_MODE) {
			Search_Control();
		}
	}
}





/************************************************
*
* 红外引脚读取辅助--少数服从多数
*
************************************************/
static uint8_t read_ir_pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    uint8_t v1 = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
    uint8_t v2 = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
    uint8_t v3 = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
    return (v1 & v2) | (v2 & v3) | (v1 & v3);
}

/************************************************
*
* 自动返航
*
************************************************/
void auto_back(void)
{
	uint8_t left, right, center;
	uint32_t now;
	
    if (g_return_state != RETURN_BACKING) 
		return;
	
    left  = read_ir_pin(GPIOB, IR_LEFT);
    right = read_ir_pin(GPIOB, IR_RIGHT);
    center = read_ir_pin(GPIOA, IR_CENTER);
    
	now = get_tick();
	
	
    // 1. 中心触发 -> 立即停车（最高优先级）
    if (center == 1) {
        g_car_state = CAR_STOP;
        turn_direction = 0;
		g_return_state = RETURN_ARRIVED;
        return;
    }

    // 2. 如果正在转向且未超过最小转向时间，则保持原转向状态
    if (turn_direction != 0 && (now - turn_start_time < 160)) {
        // 维持之前的转向状态
        g_car_state = (turn_direction == 1) ? CAR_RIGHT : CAR_LEFT;

        return;
    }

    // 3. 转向时间到，重新评估当前偏差
    turn_direction = 0; // 清除转向标记

    if (left == 1 && right == 1) {
        g_car_state = CAR_BACKWARD;
    } 
    else if (left == 1 && right == 0) {
        // 右边没信号 -> 车尾偏右，需要右转
        g_car_state = CAR_RIGHT;
        turn_direction = 1;
        turn_start_time = now;
    } 
    else if (left == 0 && right == 1) {
        g_car_state = CAR_LEFT;
        turn_direction = 2;
        turn_start_time = now;
    } 
    else {
        // 全无信号 -> 向右搜索
        g_car_state = CAR_RIGHT;
        turn_direction = 1;
        turn_start_time = now;
    }
}

/************************************************
*
* 小车速度控制
*
************************************************/
void Speed_Control(void)
{
    uint32_t now = get_tick();
    uint8_t target = g_car_state;          // 目标状态（由其他逻辑设置）
    uint16_t on_time, off_time;

    // 根据目标动作选择不同的间歇参数
    if (target == CAR_FORWARD || target == CAR_BACKWARD) {
        on_time = MOVE_ON_TIME;
        off_time = MOVE_OFF_TIME;
    } else if (target == CAR_LEFT || target == CAR_RIGHT) {
        on_time = TURN_ON_TIME;
        off_time = TURN_OFF_TIME;
    } else {
        // 停止状态：直接输出停止，重置状态机
        if (actual_state != CAR_STOP) {
            actual_state = CAR_STOP;
            Car_Control(actual_state);
            speed_step = 0;
            last_toggle_time = now;
        }
        return;
    }

    // 间歇控制
    if (speed_step == 0) {  // 运动阶段
        if (now - last_toggle_time >= on_time) {
            last_toggle_time = now;
            speed_step = 1;
            actual_state = CAR_STOP;
            Car_Control(actual_state);
        } else {
            if (actual_state != target) {
                actual_state = target;
                Car_Control(actual_state);
            }
        }
    } else {  // 停止阶段
        if (now - last_toggle_time >= off_time) {
            last_toggle_time = now;
            speed_step = 0;
            actual_state = target;
            Car_Control(actual_state);
        } else {
            if (actual_state != CAR_STOP) {
                actual_state = CAR_STOP;
                Car_Control(actual_state);
            }
        }
    }
}

/************************************************
* 
*主函数
*
************************************************/
int main(void)
{
    Object_t obj;

	SystemInit();
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    delay_init();

    Usart1_Init(115200);
    //Usart3_Init(9600);
	//TIM2_Init();

    //Servo_Init();
    //LED_Init();
	//ir_gpio_init();
    //Motor_Init();
	//HX711_Init();

    //Lock_LED(0);
	
	//delay_ms(1000);
	//Get_Maopi();
	
	//BT_SendString("hello\r\n");
	printf("hello world\r\n");
	
    while (1) {
		//蓝牙数据解析于处理
		//BT_ProcessReceivedData();

        // 自动巡检
		//auto_cruise(obj);
		
		//紫外线灯带的控制
		//led_switch();

        // 重量检测
        //Weight_Monitor();

		//自动返航
		//auto_back();
		
        // 小车运动
		//Speed_Control();
		
		
		if (Uart_GetFrameFlag()) {
			Uart_ClearFrameFlag();

			if (parse_frame(Uart_GetFrameData(), &obj, 1)) {
				//printf("cx:%.1f, cy:%.1f\r\n", obj.cx, obj.cy);
				printf("label=%s cx=%.1f cy=%.1f w=%d h=%d score=%.2f\r\n",
                obj.label, obj.cx, obj.cy, obj.w, obj.h, obj.score);
			}
		}
		delay_ms(20);
    }
}
