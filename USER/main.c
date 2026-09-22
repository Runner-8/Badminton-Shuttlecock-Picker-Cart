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
#define LASER_OFFSET_X       14 //0
#define LASER_OFFSET_Y       62 //0

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
#define DEAD_X               16 //12
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
#define IR_RIGHT 		GPIO_Pin_14
#define IR_LEFT  		GPIO_Pin_8
#define IR_CENTER 		GPIO_Pin_15

static uint32_t turn_start_time = 0;

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
#define DEAD_BIG_X 24 //20
#define DEAD_BIG_Y 20 //20
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
        if (fabs(err_x) > DEAD_BIG_X || fabs(err_y) > DEAD_BIG_Y) {
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


    //方案2：v = KP * err * (1 + 0.01 * abs(err)) 非线性比例计算，误差越大，速度越快，误差越小，速度越慢
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

    if(err_x > DEAD_BIG_X)
    {
        //g_car_state = CAR_RIGHT;
#if MOTOR_MODE_PWM
        Motor_SetSpeed(20, 20, -30, -30);
#else
        Motor_SetSpeed(1, 1, -1, -1);
#endif
        //printf("right\r\n");
    }
    else if(err_x < -DEAD_BIG_X)
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
        if(err_y > DEAD_BIG_Y)
        {
            //g_car_state = CAR_BACKWARD;

#if MOTOR_MODE_PWM
            Motor_SetSpeed(-60, -60, -70, -70);
#else
            Motor_SetSpeed(-1, -1, -1, -1);
#endif
            //printf("backward\r\n");
        }
        else if(err_y < -DEAD_BIG_Y)
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

    // 使用与 Track() 相同的有效中心：图像中心 + 激光偏移
    dx = abs((obj->cx + TARGET_OFFSET_X) - (IMAGE_CENTER_X + LASER_OFFSET_X));
    dy = abs((obj->cy + TARGET_OFFSET_Y) - (IMAGE_CENTER_Y + LASER_OFFSET_Y));

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
             Motor_SetSpeed(80, 80, 80, 80);
			   //printf("forward\r\n");
             break;
         case CAR_BACKWARD:
             Motor_SetSpeed(-80, -80, -80, -80);
		       //printf("backward\r\n");
             break;
         case CAR_LEFT:
             Motor_SetSpeed(-100, -100, 100, 100);
			   //printf("left\r\n");
             break;
         case CAR_RIGHT:
             Motor_SetSpeed(100, 100, -100, -100);
			   //printf("right\r\n");
             break;
         case CAR_STOP:
         default:
             Motor_SetSpeed(0, 0, 0, 0);
			   //printf("stop\r\n");
             break;
     }
 }

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


// void turntable_init(void)
// {
//     GPIO_InitTypeDef GPIO_InitStructure;

//     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

//     GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
//     GPIO_Init(GPIOA, &GPIO_InitStructure);
	
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
//     GPIO_Init(GPIOA, &GPIO_InitStructure);

//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
//     GPIO_Init(GPIOB, &GPIO_InitStructure);

// 	GPIO_ResetBits(GPIOA, GPIO_Pin_11);
// }

/************************************************
*
*主函数
*
************************************************/
#define FUNC        1

#if FUNC == 0

int main(void)
{
    Object_t obj;
    uint8_t flag = 0;
    uint8_t last_pin_state = 0;
    uint8_t last_pb12 = 0;
    uint8_t last_pb13 = 0;

	//SystemInit();

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    SysTick_Init();

    Usart1_Init(115200);
    //Usart3_Init(9600);

    Motor_Init();
    //Fan_Init();
	//ir_gpio_init();

    turntable_init();

	//BT_SendString("hello\r\n");
	printf("hello world\r\n");

    while (1) {
		//蓝牙数据解析于处理
		//BT_ProcessReceivedData();

		// if (Uart_GetFrameFlag()) {
		// 	Uart_ClearFrameFlag();

		// 	if (parse_frame(Uart_GetFrameData(), &obj, 1)) {
		// 		//printf("cx:%.1f, cy:%.1f\r\n", obj.cx, obj.cy);
		// 		// printf("label=%s cx=%.1f cy=%.1f w=%d h=%d\r\n",
        //         // obj.label, obj.cx, obj.cy, obj.w, obj.h);

        //         Track(&obj);
		// 	}
		// }

        uint8_t current_state = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
        uint8_t pb12 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);  // 读取 PB12（顶部红外）
        uint8_t pb13 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);

        if(last_pin_state && !current_state)
        {
            flag ^= 1;
            delay_ms(20);
        }
        last_pin_state = current_state;

        if(last_pb12 && !pb12)            // PB12 上升沿（0→1）
        {
            last_pb12 = 0;
            delay_ms(20);
            if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == Bit_RESET)
            {
                printf("PB12 IR Top Triggered!\r\n");
                flag = 1;
            }
        }

        if(last_pb13 && !pb13)            // PB13 上升沿（0→1）
        {
            last_pb13 = 0;
            delay_ms(20);
            if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == Bit_RESET)
            {
                printf("PB13 IR Funnel Triggered!\r\n");
                flag = 0;
            }
        }

        {
            GPIO_ResetBits(GPIOA, GPIO_Pin_11);
        }
 
        if(last_pb12 && !pb12)
        {
            last_pb12 = 0;
            delay_ms(20);
            if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == Bit_RESET)
            {
                printf("PB12 IR Top Triggered!\r\n");
                flag = 1;
            }
        }
 
        if(last_pb13 && !pb13)
        {
            last_pb13 = 0;
            delay_ms(20);
            if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == Bit_RESET)
            {
                printf("PB13 IR Funnel Triggered!\r\n");
                flag = 0;
            }
        }
 
        if(pb12) last_pb12 = 1;
        if(pb13) last_pb13 = 1;

        if(flag)
        {
            GPIO_SetBits(GPIOA, GPIO_Pin_11);
        }
        else
        {
            GPIO_ResetBits(GPIOA, GPIO_Pin_11);
        }


        // 小车运动
		//Speed_Control();
		//Car_Control(g_car_state);
    }
}



#else

void System_Init(void);
void ir_gpio_init(void);
void turntable_gpio_init(void);
void ManualMode_Run(void);
void AutoMode_Run(Object_t *obj);
uint8_t Collect_SubStateMachine(void);
uint8_t IR_Top_Detected(void);
uint8_t IR_Funnel_Detected(void);
uint8_t Timer_Elapsed(uint32_t *start, uint32_t timeout_ms);
void Fan_Stop(void);
void Turntable_Rotate(void);
uint8_t AllBucketsFull(void);
void Patrol_Move(void);

// ============ 状态定义 ============
typedef enum {
    MODE_MANUAL = 0,
    MODE_AUTO   = 1
} SystemMode_t;

typedef enum {
    AUTO_NONE = 0,
    AUTO_PATROL  = 1,
    AUTO_TRACK   = 2,
    AUTO_COLLECT = 3,
    AUTO_RETURN  = 4
} AutoTask_t;

typedef enum {
    COLLECT_SUCK_BOTH = 0,
    COLLECT_DETECTED = 1,
    COLLECT_SUCK_HORIZ = 2,
    COLLECT_DROP = 3,
    COLLECT_CHECK_BUCKET = 4,
    COLLECT_ROTATE = 5,
    COLLECT_DONE = 6
} CollectSubState_t;

// ============ 全局状态变量 ============
AutoTask_t        g_auto_task  = AUTO_PATROL;   // 默认巡检
CollectSubState_t g_collect_st = COLLECT_SUCK_BOTH;

uint8_t g_bucket_count[3] = {0};    // 三个桶的球数
uint8_t g_current_bucket  = 0;      // 当前桶索引
#define BALLS_PER_BUCKET   2        // 每桶容量
#define BUCKET_COUNT       3

// ============ 定时器相关 ============
static uint32_t track_lost_time = 0;      // 跟踪丢失开始时间
static uint32_t collect_start_time = 0;   // 吸取阶段开始时间

/************************************************
* 定时参数
************************************************/
#define TRACK_TIMEOUT_MS            3000          // 跟踪超时 3秒
#define COLLECT_TOP_TIMEOUT_MS      3000          // 吸取顶部超时 3秒
#define COLLECT_FUNNEL_TIMEOUT_MS   5000          // 吸取漏斗超时 5秒

uint8_t patrol_step = 0;
uint32_t patrol_timer = 0;

uint8_t g_returning = 0;


// ============ 硬件初始化 ============
void ir_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void turntable_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOA, GPIO_Pin_11);
}

static uint8_t read_ir_pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    uint8_t v1 = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
    uint8_t v2 = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
    uint8_t v3 = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
    return (v1 & v2) | (v2 & v3) | (v1 & v3);
}

uint8_t Return_ToBase(void)
{
    // 红外循迹返航：左红外=PB0, 右红外=PB1, 中间红外=PA3

    uint8_t left, right, center;
    uint32_t now = get_tick();
 
    left   = read_ir_pin(GPIOB, GPIO_Pin_14);   // PB14 左红外
    right  = read_ir_pin(GPIOA, GPIO_Pin_8);   // PB8 右红外
    center = read_ir_pin(GPIOB, GPIO_Pin_15);    // PA15 中间红外

    //printf("left: %d, right: %d, center: %d\r\n", left, right, center);
 
    // 1. 中心触发 -> 到达基地，停车
    if (center == 1) {
        g_car_state = CAR_STOP;
        turn_direction = 0;
        Car_Control(g_car_state);
        return 1;
    }
 
    // 2. 转向中且未到最小转向时间，保持
    if (turn_direction != 0 && (now - turn_start_time < 160)) {
        g_car_state = (turn_direction == 1) ? CAR_RIGHT : CAR_LEFT;
        Car_Control(g_car_state);
        return 0;
    }
 
    // 3. 转向时间到，重新评估
    turn_direction = 0;
 
    if (left == 1 && right == 1) {
        g_car_state = CAR_BACKWARD;
    }
    else if (left == 1 && right == 0) {
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
        g_car_state = CAR_RIGHT;
        turn_direction = 1;
        turn_start_time = now;
    }
 
    Car_Control(g_car_state);
    return 0;
}

void System_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init();
    Usart1_Init(115200);
	Usart3_Init(9600);

    Motor_Init();
    Fan_Init();
    ir_gpio_init();
    turntable_gpio_init();
	Turntable_Rotate();

    Motor_SetSpeed(0, 0, 0, 0);
    
    printf("System Init OK\r\n");
    //BT_SendString("System Ready\r\n");
}

void ManualMode_Run(void)
{
    if (g_returning) {
        // 返航中，持续调用直到到达
        if (Return_ToBase()) {
            g_returning = 0;
            Motor_SetSpeed(0, 0, 0, 0);
            Fan_Stop();
            for (int i = 0; i < BUCKET_COUNT; i++) {
                g_bucket_count[i] = 0;
            }
            g_current_bucket = 0;
            patrol_step = 0;
            stop_locked = 0;
            track_lost_time = 0;
            printf("return Complete!\r\n");
        }

    } else if (g_suction) {
        //printf("suction on\r\n");
		Car_Control(g_car_state);
		
        if (Collect_SubStateMachine()) {
			g_car_state = CAR_STOP;
            g_suction = 0;
            //在添加一个数据上报的功能
            BT_SendFrame(CMD_SUCK, &g_suction, 1);
            //printf("suction update\r\n");
            if (AllBucketsFull()) {
                g_returning = 1;
                turn_direction = 0;
               // printf("full!\r\n");
            }
        }
    } else {
        //printf("suction off\r\n");
        Fan_Stop();
        g_collect_st = COLLECT_SUCK_BOTH;
        //复位各个标志位
        Car_Control(g_car_state);
    }
}

//uint32_t time_aptrol = 0;

// ============ 自动模式：任务调度 ============
void AutoMode_Run(Object_t *obj)
{
    uint8_t task_done = 0;

    switch (g_auto_task)
    {
        case AUTO_NONE:
            break;

        case AUTO_PATROL:
            //自动巡检时开启风扇，保持转速20%PWM低速转动
            Fan_SetSpeed(FAN_CH1, 50);
            Fan_SetSpeed(FAN_CH2, 50);

            // 检查是否有新视觉帧
            if (Uart_GetFrameFlag()) {
                //printf("Uart_GetFrameFlag\r\n");
                Uart_ClearFrameFlag();
                if (parse_frame(Uart_GetFrameData(), obj, 1)) {
                    Motor_SetSpeed(0, 0, 0, 0);
                    g_auto_task = AUTO_TRACK;
                    track_lost_time = 0;
                    stop_locked = 0; 
                    break;
                }
            } else {
                Patrol_Move();
            }
            break;

        case AUTO_TRACK:
            if (Uart_GetFrameFlag()) {
                //printf("Uart_GetFrameFlag\r\n");
                Uart_ClearFrameFlag();
                if (parse_frame(Uart_GetFrameData(), obj, 1)) {
                    Track(obj);
                    track_lost_time = 0;

                    if (Check_Lock(obj)) {
                        Motor_SetSpeed(0, 0, 0, 0);
                        g_auto_task = AUTO_COLLECT;
                        g_collect_st = COLLECT_SUCK_BOTH;
                        collect_start_time = 0;
                        //printf("Collect Start\r\n");
						//time_aptrol = 0;
                    }
                } else {
                    // 解析失败，检查超时
                    if (Timer_Elapsed(&track_lost_time, TRACK_TIMEOUT_MS)) {
                        g_auto_task = AUTO_PATROL;
                        stop_locked = 0;
                        //printf("Track Timeout0\r\n");
                    }
                }
            } else {
                if (Timer_Elapsed(&track_lost_time, TRACK_TIMEOUT_MS)) {
                    g_auto_task = AUTO_PATROL;
                    stop_locked = 0;
                    //printf("Track Timeout1\r\n");
                } else {
                    Track(obj);
                }
            }
            break;

        case AUTO_COLLECT:
            task_done = Collect_SubStateMachine();
            if (task_done) {
                Motor_SetSpeed(0, 0, 0, 0);
				Fan_Stop();
				//time_aptrol = 0;
                if (AllBucketsFull()) {
					Fan_Stop();
                    g_auto_task = AUTO_RETURN;
                    //return_state = 0;
                    //printf("Collect Done\r\n");
                } else {
					// if (Timer_Elapsed(&time_aptrol, 4000)) {
					// 	g_auto_task = AUTO_PATROL;
					// 	patrol_step = 0;
					// }

                    Fan_Stop();
                    delay_ms(20000);
                    g_auto_task = AUTO_PATROL;
					patrol_step = 0;
                    //printf("Collect Timeout\r\n");
                }
            }
            break;

        case AUTO_RETURN:
			//printf("return\r\n");
            task_done = Return_ToBase();
            if (task_done) {
                Motor_SetSpeed(0, 0, 0, 0);
                Fan_Stop();
                // 切换到巡检模式，开始新一轮
                for (int i = 0; i < BUCKET_COUNT; i++) {
                    g_bucket_count[i] = 0;
                }
                g_current_bucket = 0;
                g_auto_task = AUTO_NONE;
                patrol_step = 0;
                stop_locked = 0;
                track_lost_time = 0;
                printf("Mission Complete!\r\n");
            }
            break;
    }
}

// ============ 巡检移动 ============
void Patrol_Move(void)
{
    // 简单巡线：按固定轨迹移动（前进→右转→前进→右转，循环）
    // 实际应结合巡线传感器或预设路径
    uint32_t now = get_tick();

    switch (patrol_step) {
        case 0:  // 前进
            Motor_SetSpeed(80, 80, 80, 80);
            if (now - patrol_timer > 1500) {  // 前进2秒
                patrol_timer = now;
                patrol_step = 1;
            }
            break;
        case 1:  // 右转
            Motor_SetSpeed(100, 100, -100, -100);
            if (now - patrol_timer > 1000) {   // 右转0.8秒
                patrol_timer = now;
                patrol_step = 2;
            }
            break;
        case 2:  // 前进
            Motor_SetSpeed(80, 80, 80, 80);
            if (now - patrol_timer > 1500) {  // 前进2秒
                patrol_timer = now;
                patrol_step = 3;
            }
            break;
        case 3:  // 左转
            Motor_SetSpeed(-100, -100, 100, 100);
            if (now - patrol_timer > 1000) {   // 左转0.8秒
                patrol_timer = now;
                patrol_step = 0;
            }
            break;
    }
}

// ============ 吸取子状态机 ============
uint8_t Collect_SubStateMachine(void)
{
    switch (g_collect_st)
    {
        case COLLECT_SUCK_BOTH:
			//printf("fan start\r\n");
            // 双风扇同速开启（垂直吸起）
			//这个时间在这个地方置为0，代表着超时处理并不使用，风扇一直开启，知道球被吸起，触发红外为止
			collect_start_time = 0;
            Fan_SetSpeed(FAN_CH1, 60);   // 垂直风扇
            Fan_SetSpeed(FAN_CH2, 90);   // 水平风扇（同速）
            if (IR_Top_Detected()) {
                collect_start_time = get_tick();
                g_collect_st = COLLECT_DETECTED;
                //printf("top\r\n");
            } else if (Timer_Elapsed(&collect_start_time, COLLECT_TOP_TIMEOUT_MS)) {
                // 超时，放弃当前球
                Fan_Stop();
                g_collect_st = COLLECT_DONE;
                //printf("Collect Timeout1\r\n");
            }
            break;

        case COLLECT_DETECTED:
			//printf("switch speed\r\n");
            // 球到达顶部，调速：垂直减速，水平加速
            Fan_SetSpeed(FAN_CH1, 0);    // 垂直减速
            Fan_SetSpeed(FAN_CH2, 80);   // 水平加速（横向吸入）
            if (Timer_Elapsed(&collect_start_time, 200)) {
                g_collect_st = COLLECT_SUCK_HORIZ;
                //printf("Collect Timeout2\r\n");
            }
            break;

        case COLLECT_SUCK_HORIZ:
			//printf("collect_suck_horiz\r\n");
            // 等待球横向吸入漏斗
			//在这里置零会一直等待红外的信号
            collect_start_time = 0;
            if (IR_Funnel_Detected()) {
                g_collect_st = COLLECT_DROP;
                //printf("funnel\r\n");
            } else if (Timer_Elapsed(&collect_start_time, COLLECT_FUNNEL_TIMEOUT_MS)) {
                // 超时，放弃
                Fan_Stop();
                g_collect_st = COLLECT_DONE;
                //printf("Collect Timeout3\r\n");
            }
            break;

        case COLLECT_DROP:
			//printf("collect_drop\r\n");
            // 球落入桶中
            Fan_Stop();
            g_bucket_count[g_current_bucket]++;
            g_collect_st = COLLECT_CHECK_BUCKET;
		
			// printf("M:%d,T:%d,B0:%d,B1:%d,B2:%d\r\n",
			// 		g_mode,
			// 		g_auto_task,
			// 		g_bucket_count[0],
			// 		g_bucket_count[1],
			// 		g_bucket_count[2]);

			BT_SendFrame(CMD_BALLCOUNTS, g_bucket_count, 3);
            break;

        case COLLECT_CHECK_BUCKET:
			//printf("collect_check_bucket\r\n");
            if (AllBucketsFull()) {
                // 所有桶已满，直接完成（不再旋转）
                g_collect_st = COLLECT_DONE;
                //printf("Collect Done3\r\n");
            } else {
                if (g_bucket_count[g_current_bucket] >= BALLS_PER_BUCKET) {
                    g_collect_st = COLLECT_ROTATE;
                    //printf("Collect Rotate\r\n");
                } else {
                    g_collect_st = COLLECT_DONE;
                    //printf("Collect Done4\r\n");
                }
            }
            break;

        case COLLECT_ROTATE:
			delay_ms(1000);
			//printf("collect_rotate\r\n");
            Turntable_Rotate();
            g_current_bucket++;
            g_collect_st = COLLECT_DONE;
            //printf("Collect Done5\r\n");
            break;

        case COLLECT_DONE:
			//printf("colledt_done\r\n");
            g_collect_st = COLLECT_SUCK_BOTH;
            collect_start_time = 0;
            //printf("Collect Done6\r\n");
            return 1;  // 吸取任务完成
    }
    return 0;
}


// ============ 红外传感器读取 ============
uint8_t IR_Top_Detected(void)
{
    // 吸取管顶部红外：检测到球返回1，否则0
    // 根据实际传感器电平调整（有球=低电平）
    //printf("IR Top Detected: %d\r\n", GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12));
    return (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == Bit_SET);
    
}

uint8_t IR_Funnel_Detected(void)
{
    // 漏斗处红外：检测到球掉落返回1
    //printf("IR Funnel Detected: %d\r\n", GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13));
    return (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == Bit_SET);
}

// ============ 定时器工具 ============
uint8_t Timer_Elapsed(uint32_t *start, uint32_t timeout_ms)
{
    if (*start == 0) {
        *start = get_tick();
        return 0;
    }
    return ((get_tick() - *start) >= timeout_ms);
}

// ============ 风扇控制 ============
void Fan_Stop(void)
{
    Fan_SetSpeed(FAN_CH1, 0);
    Fan_SetSpeed(FAN_CH2, 0);
    //printf("Fan Stop\r\n");
}
 
// ============ 转盘控制 ============
void Turntable_Rotate(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_11);
    delay_ms(50);

    while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == Bit_RESET);

    while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == Bit_SET);
    delay_ms(30);

    GPIO_ResetBits(GPIOA, GPIO_Pin_11);
    delay_ms(200);
    //printf("Turntable Rotate 44\r\n");
}

// ============ 桶满判断 ============
uint8_t AllBucketsFull(void)
{
    return (g_bucket_count[0] >= BALLS_PER_BUCKET &&
            g_bucket_count[1] >= BALLS_PER_BUCKET &&
            g_bucket_count[2] >= BALLS_PER_BUCKET);
}

uint8_t last_mode = MODE_MANUAL;

void Status_Reset(void)
{
    // 2. 模式切换：复位所有状态（保留桶计数和索引）
    if (g_mode != last_mode) {
        Motor_SetSpeed(0, 0, 0, 0);
        Fan_Stop();
        g_auto_task      = AUTO_PATROL;
        g_collect_st     = COLLECT_SUCK_BOTH;
        track_lost_time  = 0;
        collect_start_time = 0;
        stop_locked      = 0;
        patrol_step      = 0;
        patrol_timer     = 0;
        //return_state     = 0;
        g_suction        = 0;
        g_car_state      = CAR_STOP;
        last_mode        = g_mode;
    }
}

// ============ 主函数 ============
int main(void)
{
    Object_t obj;

    System_Init();       // 所有硬件初始化
	
    while (1)
    {
        // 1. 蓝牙数据处理（始终运行，接收模式切换指令）
        BT_ProcessReceivedData();

		Status_Reset();
        
        // 2. 根据系统模式分发
        if (g_mode == MODE_MANUAL) {
             ManualMode_Run();      // 手动模式：APP直接控制
        } else {
             AutoMode_Run(&obj);    // 自动模式：任务状态机
        }
		
    }
}

#endif
