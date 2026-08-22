#include "stm32f10x.h"
#include "bsp_usart.h"
#include "PWM.h"
#include "bluetooth.h"
#include <stdlib.h>
#include "motor.h"
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
#define DEAD_X               12
#define DEAD_Y               12

/************************************************
*
*精准锁定死区
*
************************************************/
#define AIM_STOP_X           6
#define AIM_STOP_Y           6

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
//static uint32_t last_v_time = 0;
//static uint32_t last_h_time = 0;


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

//static uint8_t actual_state = CAR_STOP;   // 当前实际输出状态
//static uint32_t last_toggle_time = 0;
//static uint8_t speed_step = 0;            // 0: 运动阶段, 1: 停止阶段
//static uint32_t turn_start_time = 0;


/************************************************
*
* 自动巡检
*
************************************************/
//void Search_Control(void)
//{
//	uint32_t now = get_tick();

//    switch (search_step) {
//        case 0:
//			if (now - last_v_time < SEARCH_V_DELAY)
//				return;
//			last_v_time = now;

//            search_v = SEARCH_V_CENTER;
//            search_step = 1;
//            break;

//        case 1:
//			if (now - last_v_time < SEARCH_V_DELAY)
//				return;
//			last_v_time = now;

//            search_v = SEARCH_V_CENTER - SEARCH_V_OFFSET;
//            search_step = 2;
//            break;

//        case 2:
//			if (now - last_v_time < SEARCH_V_DELAY)
//				return;
//			last_v_time = now;

//            search_v = SEARCH_V_CENTER;
//            search_step = 3;
//            break;

//        case 3:
//			if (now - last_v_time < SEARCH_V_DELAY)
//				return;
//			last_v_time = now;

//            search_v = SEARCH_V_CENTER + SEARCH_V_OFFSET;
//            search_step = 4;
//            break;

//        case 4:
//			if (now - last_h_time < SEARCH_V_DELAY)
//				return;
//			last_h_time = now;

//            search_h += SEARCH_H_STEP;
//            if (search_h >= SEARCH_H_END) {
//                search_h = SEARCH_H_START;
//            }
//            search_step = 0;
//            break;
//    }

//    Servo_SetAngle_H(search_h);
//    Servo_SetAngle_V(search_v);
//}

/************************************************
*
*视觉跟踪
*
************************************************/
void Track_Control(Object_t *obj)
{
    float err_x;
    float err_y;
    //float delta_h;
    //float delta_v;
    //static float last_h = 0;
    //static float last_v = 0;
    float target_x;
    float target_y;

    target_x = obj->cx + TARGET_OFFSET_X;
    target_y = obj->cy + TARGET_OFFSET_Y;

    err_x = target_x - (IMAGE_CENTER_X + LASER_OFFSET_X);
    err_y = target_y - (IMAGE_CENTER_Y + LASER_OFFSET_Y);

    // if (fabs(err_x) < DEAD_X)
    //     err_x = 0;
    // if (fabs(err_y) < DEAD_Y)
    //     err_y = 0;

    // if (fabs(err_x) < AIM_STOP_X)
    //     err_x = 0;
    // if (fabs(err_y) < AIM_STOP_Y)
    //     err_y = 0;

            // X方向 左右平移（麦轮横向）
    if(err_x > DEAD_X)
    {
        g_car_state = CAR_RIGHT;   // IO输出：小车右移
        printf("right\r\n");
    }
    else if(err_x < -DEAD_X)
    {
        g_car_state = CAR_LEFT;    // IO输出：小车左移
        printf("left\r\n");
    }
    else
    {
        g_car_state = CAR_STOP;       // X方向停止

        // Y方向 前后前进后退
        if(err_y > DEAD_Y)
        {
            g_car_state = CAR_BACKWARD;     // 目标太靠近，后退
            printf("backward\r\n");
        }
        else if(err_y < -DEAD_Y)
        {
            g_car_state = CAR_FORWARD;      // 目标较远，向前靠近
            printf("forward\r\n");
        }
        else
        {
            g_car_state = CAR_STOP;
            printf("stop\r\n");
        }
    }



    // delta_h = -err_x * KP_H;
    // delta_v = err_y * KP_V;

    // delta_h = FILTER_H * delta_h + (1 - FILTER_H) * last_h;
    // delta_v = FILTER_V * delta_v + (1 - FILTER_V) * last_v;

    // last_h = delta_h;
    // last_v = delta_v;

    // if (delta_h > MAX_STEP_H)
    //     delta_h = MAX_STEP_H;
    // if (delta_h < -MAX_STEP_H)
    //     delta_h = -MAX_STEP_H;
    // if (delta_v > MAX_STEP_V)
    //     delta_v = MAX_STEP_V;
    // if (delta_v < -MAX_STEP_V)
    //     delta_v = -MAX_STEP_V;

    // current_angle_h += delta_h;
    // current_angle_v += delta_v;

    // if (current_angle_h < H_MIN)
    //     current_angle_h = H_MIN;
    // if (current_angle_h > H_MAX)
    //     current_angle_h = H_MAX;
    // if (current_angle_v < V_MIN)
    //     current_angle_v = V_MIN;
    // if (current_angle_v > V_MAX)
    //     current_angle_v = V_MAX;

    // Servo_SetAngle_H(current_angle_h);
    // Servo_SetAngle_V(current_angle_v);
}

// 在文件顶部定义滤波系数和比例系数
#define KP_X        0.95//0.82f    // X方向比例系数
#define KP_Y        0.88//0.81f    // Y方向比例系数
#define FILTER_ALPHA 0.35f   // 滤波平滑度（0~1），越小越平滑

// 静态变量保存上次滤波值（放在函数外或声明为static）
static float filtered_vx = 0;
static float filtered_vy = 0;

// 全局/静态变量，放在函数外部
#define DEAD_x 20
#define DEAD_y 20
// 目标位置变化阈值，根据视觉像素调试
#define DELTA_THRESH 8

static float last_target_x = 0;
static float last_target_y = 0;
static uint8_t stop_locked = 0;  // 停止锁定标志

void Track(Object_t *obj)
{
    // 1. 计算原始误差（像素）
    float err_x, err_y;
    int16_t lf, rf;
    float vx, vy;
	//float omega;

    err_x = (obj->cx + TARGET_OFFSET_X) - (IMAGE_CENTER_X + LASER_OFFSET_X);
    err_y = (obj->cy + TARGET_OFFSET_Y) - (IMAGE_CENTER_Y + LASER_OFFSET_Y);

        // ---- 精准停止锁定（带迟滞）----
    if (fabs(err_x) < AIM_STOP_X && fabs(err_y) < AIM_STOP_Y) {
        stop_locked = 1;                    // 进入精准范围，锁定停止
    }
    if (stop_locked) {
        if (fabs(err_x) > DEAD_x || fabs(err_y) > DEAD_y) {
            stop_locked = 0;                // 目标跑出大死区，解锁
        } else {
            Motor_SetSpeed(0, 0, 0, 0);     // 保持停止
            return;
        }
    }

    if (fabs(err_x) < DEAD_X)
        err_x = 0;
    if (fabs(err_y) < DEAD_Y)
        err_y = 0;


    //v = KP * err * (1 + 0.01 * abs(err)) 非线性比例计算，误差越大，速度越快，误差越小，速度越慢
    // 3. 比例控制：速度 = 系数 × 误差
    vx = -KP_Y * err_y;
    vy = KP_X * err_x;
//    omega = 0;   // 自旋速度，暂不启用

    // 2. 一阶低通滤波（平滑误差）
    filtered_vx = FILTER_ALPHA * vx + (1 - FILTER_ALPHA) * filtered_vx;
    filtered_vy = FILTER_ALPHA * vy + (1 - FILTER_ALPHA) * filtered_vy;

    //printf("vx:%f, vy:%f, omega:%f\r\n", filtered_vx, filtered_vy, omega);


    // 6. 逆运动学解算（得到四个轮速）
    //Kinematics_Inverse(filtered_vx, filtered_vy, omega, &lf, &lb, &rf, &rb);
    Kinematics_Differential(filtered_vx, filtered_vy, &lf, &rf);

    // 7. 再次对轮速应用死区（可选，但建议保留） SPEED_DEAD_ZONE = 30
//    lf = apply_startup_compensation(lf, SPEED_DEAD_ZONE);
//    rf = apply_startup_compensation(rf, SPEED_DEAD_ZONE);
//    lb = apply_startup_compensation(lb, SPEED_DEAD_ZONE);
//    rb = apply_startup_compensation(rb, SPEED_DEAD_ZONE);

    //printf("lf:%d, rf:%d\r\n", lf, rf);

    // 8. 驱动电机
    Motor_SetSpeed(lf, lf, rf, rf);
}

// new_target_x, new_target_y 是串口3刚收到的最新目标坐标
void Car(float new_target_x, float new_target_y)
{

	float err_x, err_y;
    float dx = fabs(new_target_x - last_target_x);
    float dy = fabs(new_target_y - last_target_y);

	//printf("1\r\n");

    // 目标位置变动很小，不更新运动状态，直接退出
    if(dx < DELTA_THRESH && dy < DELTA_THRESH)
    {
        return;
    }
    // 目标明显移动，刷新保存坐标，重新计算控制状态
    last_target_x = new_target_x;
    last_target_y = new_target_y;


    err_x = new_target_x - (IMAGE_CENTER_X + LASER_OFFSET_X);  // 你自行替换成真实误差计算：err_x = target_x - center_x
    err_y = new_target_y - (IMAGE_CENTER_Y + LASER_OFFSET_Y);  // err_y = target_y - center_y

    if(err_x > DEAD_x)
    {
        //g_car_state = CAR_RIGHT;
#if MOTOR_MODE_PWM
        Motor_SetSpeed(20, 20, -30, -30);
#else
        Motor_SetSpeed(1, 1, -1, -1);
#endif
        //printf("right\r\n");
    }
    else if(err_x < -DEAD_x)
    {
        //g_car_state = CAR_LEFT;
#if MOTOR_MODE_PWM
        Motor_SetSpeed(-40, -40, 50, 50);
#else
        Motor_SetSpeed(-1, -1, 1, 1);
#endif
        //printf("left\r\n");
    }
    else
    {
        if(err_y > DEAD_y)
        {
            //g_car_state = CAR_BACKWARD;

#if MOTOR_MODE_PWM
            Motor_SetSpeed(-60, -60, -70, -70);
#else
            Motor_SetSpeed(-1, -1, -1, -1);
#endif
            //printf("backward\r\n");
        }
        else if(err_y < -DEAD_y)
        {
            //g_car_state = CAR_FORWARD;
#if MOTOR_MODE_PWM
            Motor_SetSpeed(80, 80, 90, 90);
#else
            Motor_SetSpeed(1, 1, 1, 1);
#endif
            //printf("forward\r\n");
        }
        else
        {
            //g_car_state = CAR_STOP;
            Motor_SetSpeed(0, 0, 0, 0);
            //printf("stop\r\n");
        }
    }
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


// void Car_Control(uint8_t state)
// {
//     switch (state) {
//         case CAR_FORWARD:
//             Motor_SetSpeed(80, 80, 80, 80);
//             break;
//         case CAR_BACKWARD:
//             Motor_SetSpeed(-80, -80, -80, -80);
//             break;
//         case CAR_LEFT:
//             Motor_SetSpeed(-80, -80, 80, 80);
//             break;
//         case CAR_RIGHT:
//             Motor_SetSpeed(80, 80, -80, -80);
//             break;
//         case CAR_STOP:
//         default:
//             Motor_SetSpeed(0, 0, 0, 0);
//             break;
//     }
// }
/************************************************
*
* 自动巡检
*
************************************************/
//void auto_cruise(Object_t obj)
//{
//	if (g_cruise_enabled) {
//		if (Uart_GetFrameFlag()) {
//			Uart_ClearFrameFlag();

//			if (parse_frame(Uart_GetFrameData(), &obj, 1)) {
//				system_state = TRACK_MODE;
//				Track_Control(&obj);

//				if (Check_Lock(&obj)) {
//					Lock_LED(1);
//				}
//			} else {
//				system_state = SEARCH_MODE;
//				lock_count = 0;
//				Lock_LED(0);
//			}
//	}

//		if (system_state == SEARCH_MODE) {
//			Search_Control();
//		}
//	}
//}

/************************************************
*
* 红外引脚读取辅助--少数服从多数
*
************************************************/
//static uint8_t read_ir_pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
//{
//    uint8_t v1 = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
//    uint8_t v2 = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
//    uint8_t v3 = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
//    return (v1 & v2) | (v2 & v3) | (v1 & v3);
//}

/************************************************
*
* 自动返航
*
************************************************/
//void auto_back(void)
//{
//	uint8_t left, right, center;
//	uint32_t now;

//    if (g_return_state != RETURN_BACKING)
//		return;

//    left  = read_ir_pin(GPIOB, IR_LEFT);
//    right = read_ir_pin(GPIOB, IR_RIGHT);
//    center = read_ir_pin(GPIOA, IR_CENTER);

//	now = get_tick();


//    // 1. 中心触发 -> 立即停车（最高优先级）
//    if (center == 1) {
//        g_car_state = CAR_STOP;
//        turn_direction = 0;
//		g_return_state = RETURN_ARRIVED;
//        return;
//    }

//    // 2. 如果正在转向且未超过最小转向时间，则保持原转向状态
//    if (turn_direction != 0 && (now - turn_start_time < 160)) {
//        // 维持之前的转向状态
//        g_car_state = (turn_direction == 1) ? CAR_RIGHT : CAR_LEFT;

//        return;
//    }

//    // 3. 转向时间到，重新评估当前偏差
//    turn_direction = 0; // 清除转向标记

//    if (left == 1 && right == 1) {
//        g_car_state = CAR_BACKWARD;
//    }
//    else if (left == 1 && right == 0) {
//        // 右边没信号 -> 车尾偏右，需要右转
//        g_car_state = CAR_RIGHT;
//        turn_direction = 1;
//        turn_start_time = now;
//    }
//    else if (left == 0 && right == 1) {
//        g_car_state = CAR_LEFT;
//        turn_direction = 2;
//        turn_start_time = now;
//    }
//    else {
//        // 全无信号 -> 向右搜索
//        g_car_state = CAR_RIGHT;
//        turn_direction = 1;
//        turn_start_time = now;
//    }
//}

/************************************************
*
* 小车速度控制
*
************************************************/
#if MOTOR_MODE_PWM == 0
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
#endif
/************************************************
*
*主函数
*
************************************************/

uint32_t time;
uint8_t pwm_fan = 0;

int main(void)
{
    Object_t obj;

	//SystemInit();

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    SysTick_Init();

    Usart1_Init(115200);
    //Usart3_Init(9600);

    Motor_Init();
    //Fan_Init();
	//ir_gpio_init();

	//BT_SendString("hello\r\n");
	printf("hello world\r\n");

    while (1) {
		//蓝牙数据解析于处理
		//BT_ProcessReceivedData();

		if (Uart_GetFrameFlag()) {
			Uart_ClearFrameFlag();

			if (parse_frame(Uart_GetFrameData(), &obj, 1)) {
				//printf("cx:%.1f, cy:%.1f\r\n", obj.cx, obj.cy);
				// printf("label=%s cx=%.1f cy=%.1f w=%d h=%d\r\n",
                // obj.label, obj.cx, obj.cy, obj.w, obj.h);

                Track(&obj);
			}
		}

        // if(get_tick() - time > 5000)
        // {
        //     time = get_tick();
        //     pwm_fan += 10;
        //     if(pwm_fan > 100)
        //         pwm_fan = 0;
        //     Fan_SetSpeed(FAN_CH1, pwm_fan);
        // }

        // 小车运动
		//Speed_Control();
		//Car_Control(g_car_state);
    }
}

#define FUNC        0

#if FUNC

// ============ 状态定义 ============
typedef enum {
    MODE_MANUAL = 0,
    MODE_AUTO   = 1
} SystemMode_t;

typedef enum {
    AUTO_PATROL  = 0,
    AUTO_TRACK   = 1,
    AUTO_COLLECT = 2,
    AUTO_RETURN  = 3
} AutoTask_t;

typedef enum {
    COLLECT_APPROACH = 0,
    COLLECT_SUCK_BOTH = 1,
    COLLECT_DETECTED = 2,
    COLLECT_SUCK_HORIZ = 3,
    COLLECT_DROP = 4,
    COLLECT_CHECK_BUCKET = 5,
    COLLECT_ROTATE = 6,
    COLLECT_DONE = 7
} CollectSubState_t;

// ============ 全局状态变量 ============
AutoTask_t        g_auto_task  = AUTO_PATROL;   // 默认巡检
CollectSubState_t g_collect_st = COLLECT_APPROACH;

uint8_t g_bucket_count[3] = {0};  // 三个桶的球数
uint8_t g_current_bucket  = 0;    // 当前桶索引
#define BALLS_PER_BUCKET   5       // 每桶容量

// ============ 主函数 ============
int main(void)
{
    Object_t obj;
    
    System_Init();       // 所有硬件初始化
    
    while (1)
    {
        // 1. 蓝牙数据处理（始终运行，接收模式切换指令）
        BT_ProcessReceivedData();
        
        // 2. 根据系统模式分发
        if (g_mode == MODE_MANUAL) {
            ManualMode_Run();      // 手动模式：APP直接控制
        } else {
            AutoMode_Run(&obj);    // 自动模式：任务状态机
        }
        
        // 3. 状态同步到APP（蓝牙上报）
        Status_ReportToApp();
    }
}

void ManualMode_Run(void)
{
    Car_Control(g_car_state);
}

void AutoMode_Run(Object_t *obj)
{
    uint8_t task_done = 0;
    
    switch (g_auto_task)
    {
        // ============ 巡检 ============
        case AUTO_PATROL:
            // 先检查是否有新的视觉帧
            if (Uart_GetFrameFlag()) {
                Uart_ClearFrameFlag();
                if (parse_frame(Uart_GetFrameData(), obj, 1)) {
                    // 检测到球！立即切换状态
                    g_auto_task = AUTO_TRACK;
                }
            }
            // 没检测到球，继续巡线移动
            Patrol_Move();
            break;
            
        // ============ 跟踪 ============
        case AUTO_TRACK:
            // 先检查是否还有视觉帧
            if (Uart_GetFrameFlag()) {
                Uart_ClearFrameFlag();
                if (parse_frame(Uart_GetFrameData(), obj, 1)) {
                    // 持续跟踪
                    Track(obj);
                    
                    // 判断是否到达吸取范围
                    if (Check_Lock(obj)) {   // 目标在中心稳定N帧
                        Motor_SetSpeed(0, 0, 0, 0);          // 停车
                        g_auto_task = AUTO_COLLECT;  // ← 切换！
                        g_collect_st = COLLECT_APPROACH;
                    }
                } else {
                    // 丢失目标，回到巡检
                    g_auto_task = AUTO_PATROL;
                }
            } else {
                // 超时没收到帧，可能丢失目标
                if (Track_Timeout()) {
                    g_auto_task = AUTO_PATROL;  // ← 超时回巡检
                }
            }
            break;
            
        // ============ 吸取 ============
        case AUTO_COLLECT:
            task_done = Collect_SubStateMachine();  // 子状态机内部处理
            if (task_done) {
                if (AllBucketsFull()) {
                    g_auto_task = AUTO_RETURN;  // ← 三桶满，返航
                } else {
                    g_auto_task = AUTO_PATROL;  // ← 继续找球
                }
            }
            break;
            
        // ============ 返航 ============
        case AUTO_RETURN:
            task_done = Return_ToBase();
            if (task_done) {
                Motor_Stop();
                Fan_Stop();
                // 任务全部完成，可以闪烁LED或上报APP
            }
            break;
    }
}


uint8_t Collect_SubStateMachine(void)
{
    switch (g_collect_st)
    {
        case COLLECT_APPROACH:
            // 微调位置
            if (Approach_Ready()) {
                g_collect_st = COLLECT_SUCK_BOTH;  // ← 内部转换
            }
            break;
            
        case COLLECT_SUCK_BOTH:
            Fan_SetSpeed(FAN_CH1,  100);
            Fan_SetSpeed(FAN_CH2, 80);
            if (IR_Top_Detected()) {                // 红外检测到球到顶
                g_collect_st = COLLECT_DETECTED;     // ← 内部转换
            }
            break;
            
        case COLLECT_DETECTED:
            Fan_SetSpeed(FAN_CH1,  80);        // 垂直减速
            Fan_SetSpeed(FAN_CH2, 100);      // 水平加速
            if (Timer_Elapsed(500)) {               // 延时等球吸入
                g_collect_st = COLLECT_SUCK_HORIZ;   // ← 内部转换
            }
            break;
            
        case COLLECT_SUCK_HORIZ:
            if (IR_Funnel_Detected()) {             // 漏斗处红外检测
                g_collect_st = COLLECT_DROP;         // ← 内部转换
            }
            break;
            
        case COLLECT_DROP:
            Fan_Stop();
            g_bucket_count[g_current_bucket]++;     // 计数+1
            g_collect_st = COLLECT_CHECK_BUCKET;     // ← 内部转换
            break;
            
        case COLLECT_CHECK_BUCKET:
            if (g_bucket_count[g_current_bucket] >= BALLS_PER_BUCKET) {
                g_collect_st = COLLECT_ROTATE;       // ← 桶满，旋转
            } else {
                g_collect_st = COLLECT_DONE;         // ← 桶未满，结束
            }
            break;
            
        case COLLECT_ROTATE:
            Turntable_Rotate(1);
            g_current_bucket++;
            g_collect_st = COLLECT_DONE;             // ← 内部转换
            break;
            
        case COLLECT_DONE:
            g_collect_st = COLLECT_APPROACH;         // 重置子状态
            return 1;  // ← 返回1告诉上层：吸取任务完成，可以切换大状态了
    }
    return 0;  // 还没完成，继续
}

#endif
