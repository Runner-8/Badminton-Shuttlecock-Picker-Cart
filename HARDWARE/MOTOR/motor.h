#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"
#include "stm32f10x.h"                  // Device header
#include "motor.h"

#define MOTOR_MODE_PWM          1
#define MOTOR_DERIVE_L298N      1
#define PWM_MAX                 100
#define PWM_MIN_EFFECTIVE       80
#define PWM_MAX_DUTY			2500U //100-10KHZ  200-5KHZ  1000-1KHZ  2000-500HZ 2500-400HZ 4000-250HZ  10000-100HZ
#define PRESCALER				72
#define LIMIT(x, min, max)  	((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#if MOTOR_MODE_PWM == 0

// ---------- 引脚宏定义 ----------
// 左前轮
#define MOTOR_LF_IN1_PORT  GPIOB
#define MOTOR_LF_IN1_PIN   GPIO_Pin_6
#define MOTOR_LF_IN2_PORT  GPIOB
#define MOTOR_LF_IN2_PIN   GPIO_Pin_7

// 左后轮
#define MOTOR_LB_IN1_PORT  GPIOB
#define MOTOR_LB_IN1_PIN   GPIO_Pin_8
#define MOTOR_LB_IN2_PORT  GPIOB
#define MOTOR_LB_IN2_PIN   GPIO_Pin_9

// 右后轮
#define MOTOR_RB_IN1_PORT  GPIOA
#define MOTOR_RB_IN1_PIN   GPIO_Pin_6
#define MOTOR_RB_IN2_PORT  GPIOA
#define MOTOR_RB_IN2_PIN   GPIO_Pin_7

// 右前轮
#define MOTOR_RF_IN1_PORT  GPIOB
#define MOTOR_RF_IN1_PIN   GPIO_Pin_0
#define MOTOR_RF_IN2_PORT  GPIOB
#define MOTOR_RF_IN2_PIN   GPIO_Pin_1

// ---------- 时钟宏 ----------
#define RCC_GPIO_LF  RCC_APB2Periph_GPIOB
#define RCC_GPIO_LB  RCC_APB2Periph_GPIOB
#define RCC_GPIO_RB  RCC_APB2Periph_GPIOA
#define RCC_GPIO_RF  RCC_APB2Periph_GPIOB

#endif


#if MOTOR_DERIVE_L298N
// ---------- 电机与定时器通道映射（L298N：1 PWM + 2 GPIO) ----------
// 结构体：{ TIMx, PWM通道, PWM_GPIO端口, PWM引脚, IN1_GPIO端口, IN1引脚, IN2_GPIO端口, IN2引脚 }
typedef struct {
    TIM_TypeDef *tim;
    uint8_t ch_pwm;
    GPIO_TypeDef *gpio_pwm;
    uint16_t pin_pwm;
    GPIO_TypeDef *gpio_in1;
    uint16_t pin_in1;
    GPIO_TypeDef *gpio_in2;
    uint16_t pin_in2;
} MotorConfig_t;
#else
// ---------- 引脚与定时器通道映射表 ----------
// 每个电机用两个 PWM 通道（INA, INB）
// 结构体：{ TIMx, 通道INA, 通道INB, GPIO端口INA, GPIO引脚INA, GPIO端口INB, GPIO引脚INB }
typedef struct {
    TIM_TypeDef *tim;
    uint8_t ch_ina;
    uint8_t ch_inb;
    GPIO_TypeDef *gpio_ina;
    uint16_t pin_ina;
    GPIO_TypeDef *gpio_inb;
    uint16_t pin_inb;
} MotorConfig_t;
#endif

// void LED_Init(void);
// void ir_gpio_init(void);
// void Motor_Init(void);          // 初始化GPIO


void Motor_GPIO_Init(void);
void Motor_PWM_Init(void);
void Motor_Init(void);
void Motor_Stop(void);          // 全停
void Motor_Forward(void);       // 前进
void Motor_Backward(void);      // 后退
void Motor_Left(void);          // 左转（原地左旋）
void Motor_Right(void);         // 右转（原地右旋）
static void Motor_SetSingle(uint8_t id, int16_t speed);
void Motor_SetSpeed(int16_t lf, int16_t lb, int16_t rf, int16_t rb);
void Kinematics_Inverse(float vx, float vy, float omega, int16_t *lf, int16_t *lb, int16_t *rf, int16_t *rb);
void Kinematics_Differential(float vx, float vy, int16_t *left, int16_t *right);
int16_t remap_to_effective_range(int16_t v, int16_t min_eff, int16_t max_eff);

#endif

