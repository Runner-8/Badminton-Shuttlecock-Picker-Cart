#include "stm32f10x.h"                  // Device header
#include "motor.h"
#include "bluetooth.h"
#include <math.h>

#if MOTOR_DERIVE_L298N
// ============ L298N 电机配置表 ============
// 4个电机：每电机 1路PWM(TIM3) + 2路GPIO方向
// 左前: TIM3 CH1(PA6) PWM,  IN1=PA2, IN2=PA3
// 左后: TIM3 CH2(PA7) PWM,  IN1=PA4, IN2=PA5
// 右前: TIM3 CH3(PB0) PWM,  IN1=PB6, IN2=PB7
// 右后: TIM3 CH4(PB1) PWM,  IN1=PB8, IN2=PB9
static const MotorConfig_t motor_cfg[4] = {
    // 左前轮
    { TIM3, 1, GPIOA, GPIO_Pin_6,  GPIOA, GPIO_Pin_2, GPIOA, GPIO_Pin_3 },
    // 左后轮
    { TIM3, 2, GPIOA, GPIO_Pin_7,  GPIOA, GPIO_Pin_4, GPIOA, GPIO_Pin_5 },
    // 右前轮
    { TIM3, 3, GPIOB, GPIO_Pin_0,  GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7 },
    // 右后轮
    { TIM3, 4, GPIOB, GPIO_Pin_1,  GPIOB, GPIO_Pin_8, GPIOB, GPIO_Pin_9 }
};
#else
static const MotorConfig_t motor_cfg[4] = {
    // 左前轮：TIM4 CH1 (PB6) 和 CH2 (PB7)
    { TIM4, 1, 2, GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7 },
    // 左后轮：TIM4 CH3 (PB8) 和 CH4 (PB9)
    { TIM4, 3, 4, GPIOB, GPIO_Pin_8, GPIOB, GPIO_Pin_9 },
    // 右前轮：TIM3 CH1 (PA6) 和 CH2 (PA7)
    { TIM3, 1, 2, GPIOA, GPIO_Pin_6, GPIOA, GPIO_Pin_7 },
    // 右后轮：TIM3 CH3 (PB0) 和 CH4 (PB1)
    { TIM3, 3, 4, GPIOB, GPIO_Pin_0, GPIOB, GPIO_Pin_1 }
};
#endif

#if MOTOR_DERIVE_L298N
// ---------- GPIO 初始化 ----------
void Motor_GPIO_Init(void)
{
    uint8_t i;
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能 GPIOA, GPIOB 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    for (i = 0; i < 4; i++) {
        const MotorConfig_t *m = &motor_cfg[i];

        // PWM引脚：复用推挽输出
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Pin   = m->pin_pwm;
        GPIO_Init(m->gpio_pwm, &GPIO_InitStructure);

        // 方向引脚 IN1, IN2：通用推挽输出
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Pin  = m->pin_in1 | m->pin_in2;
        GPIO_Init(m->gpio_in1, &GPIO_InitStructure);

        // 初始全部拉低（停止）
        GPIO_ResetBits(m->gpio_in1, m->pin_in1);
        GPIO_ResetBits(m->gpio_in2, m->pin_in2);
    }
}
#else
// ---------- 初始化 ----------
void Motor_GPIO_Init(void) 
{
	uint8_t i;
	
    GPIO_InitTypeDef GPIO_InitStructure;
    // 使能 GPIOA, GPIOB 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#if MOTOR_MODE_PWM
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
#else
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
#endif

    // 遍历所有电机引脚，统一配置
    for (i = 0; i < 4; i++) {
        const MotorConfig_t *m = &motor_cfg[i];
        GPIO_InitStructure.GPIO_Pin = m->pin_ina | m->pin_inb;
        GPIO_Init(m->gpio_ina, &GPIO_InitStructure);
        // 初始全部拉低
        GPIO_ResetBits(m->gpio_ina, m->pin_ina);
        GPIO_ResetBits(m->gpio_inb, m->pin_inb);
    }

}
#endif

#if MOTOR_MODE_PWM

#if MOTOR_DERIVE_L298N
// ---------- PWM 初始化（仅 TIM3，4通道） ----------
void Motor_PWM_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 使能 TIM3 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseStructure.TIM_Period        = PWM_MAX_DUTY - 1;
    TIM_TimeBaseStructure.TIM_Prescaler     = PRESCALER - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    // TIM3 四个通道
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_Cmd(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
}
#else

void Motor_PWM_Init(void) 
{    
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
	
    // 使能 TIM3 和 TIM4 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = PWM_MAX_DUTY - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    // 初始化 TIM3 和 TIM4（参数相同）
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    // 配置 TIM3 所有通道
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // 配置 TIM4 所有通道
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // 使能定时器
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_ARRPreloadConfig(TIM4, ENABLE);

}
#endif

//int16_t apply_startup_compensation(int16_t speed, int16_t threshold) 
//{
//    if (speed == 0) return 0;
//    if (speed > 0) {
//        return (speed < threshold) ? threshold : speed;
//    } else {
//        return (speed > -threshold) ? -threshold : speed;
//    }
//}


// 内部函数：单个轮子做低速死区重映射
int16_t remap_to_effective_range(int16_t v, int16_t min_eff, int16_t max_eff)
{
    if (v == 0) return 0;
    int16_t sign = (v > 0) ? 1 : -1;
    int16_t abs_v = (v > 0) ? v : -v;
    if (abs_v > max_eff) abs_v = max_eff;
    int32_t mapped = min_eff + (int32_t)(max_eff - min_eff) * abs_v / max_eff;
    return (int16_t)(sign * mapped);
}

// int16_t remap_to_effective_range(int16_t v, int16_t min_eff, int16_t max_eff)
// {
//     if (v == 0) return 0;
//     int16_t sign = (v > 0) ? 1 : -1;
//     int16_t abs_v = (v > 0) ? v : -v;
//     if (abs_v > max_eff) abs_v = max_eff;

//     // 改前：线性映射
//     // int32_t mapped = min_eff + (int32_t)(max_eff - min_eff) * abs_v / max_eff;

//     // 改后：小值也保证有足够力度（非线性映射）
//     // 0~40 映射到 70~85, 40~100 映射到 85~100
//     int32_t mapped;
//     if (abs_v <= 40) {
//         mapped = min_eff + (int32_t)(85 - min_eff) * abs_v / 40;
//     } else {
//         mapped = 85 + (int32_t)(max_eff - 85) * (abs_v - 40) / 60;
//     }
//     return (int16_t)(sign * mapped);
// }
#endif

// ---------- 对外接口 ----------
void Motor_Init(void)
{
    Motor_GPIO_Init();

#if MOTOR_MODE_PWM
    Motor_PWM_Init();
#endif
}

// /**
//   * @brief  所有电机停止
//   */

#if MOTOR_MODE_PWM == 0

void Motor_Stop(void)
{
    // 左侧两个电机
    GPIO_ResetBits(MOTOR_LF_IN1_PORT, MOTOR_LF_IN1_PIN | MOTOR_LF_IN2_PIN);
    GPIO_ResetBits(MOTOR_LB_IN1_PORT, MOTOR_LB_IN1_PIN | MOTOR_LB_IN2_PIN);

    // 右侧两个电机
    GPIO_ResetBits(MOTOR_RB_IN1_PORT, MOTOR_RB_IN1_PIN | MOTOR_RB_IN2_PIN);
    GPIO_ResetBits(MOTOR_RF_IN1_PORT, MOTOR_RF_IN1_PIN | MOTOR_RF_IN2_PIN);
}

/**
  * @brief  前进：所有电机正转
  * @note   正转定义为 IN1=1, IN2=0（可根据实际接线调整）
  */
void Motor_Forward(void)
{
    // 左侧两个电机正转
    GPIO_WriteBit(MOTOR_LF_IN1_PORT, MOTOR_LF_IN1_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_LF_IN1_PORT, MOTOR_LF_IN2_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_LB_IN1_PORT, MOTOR_LB_IN1_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_LB_IN1_PORT, MOTOR_LB_IN2_PIN, Bit_RESET);

    // 右侧两个电机正转
    GPIO_WriteBit(MOTOR_RB_IN1_PORT, MOTOR_RB_IN1_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_RB_IN1_PORT, MOTOR_RB_IN2_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_RF_IN1_PORT, MOTOR_RF_IN1_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_RF_IN1_PORT, MOTOR_RF_IN2_PIN, Bit_RESET);
}

/**
  * @brief  后退：所有电机反转
  * @note   反转为 IN1=0, IN2=1
  */
void Motor_Backward(void)
{
    // 左侧两个电机反转
    GPIO_WriteBit(MOTOR_LF_IN1_PORT, MOTOR_LF_IN1_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_LF_IN1_PORT, MOTOR_LF_IN2_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_LB_IN1_PORT, MOTOR_LB_IN1_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_LB_IN1_PORT, MOTOR_LB_IN2_PIN, Bit_SET);
    
    // 右侧两个电机反转
    GPIO_WriteBit(MOTOR_RB_IN1_PORT, MOTOR_RB_IN1_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_RB_IN1_PORT, MOTOR_RB_IN2_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_RF_IN1_PORT, MOTOR_RF_IN1_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_RF_IN1_PORT, MOTOR_RF_IN2_PIN, Bit_SET);
}

/**
  * @brief  左转（原地左旋）：左侧电机反转，右侧电机正转
  */
void Motor_Left(void)
{
    // 左侧电机反转
    GPIO_WriteBit(MOTOR_LF_IN1_PORT, MOTOR_LF_IN1_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_LF_IN1_PORT, MOTOR_LF_IN2_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_LB_IN1_PORT, MOTOR_LB_IN1_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_LB_IN1_PORT, MOTOR_LB_IN2_PIN, Bit_SET);
    
    // 右侧电机正转
    GPIO_WriteBit(MOTOR_RB_IN1_PORT, MOTOR_RB_IN1_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_RB_IN1_PORT, MOTOR_RB_IN2_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_RF_IN1_PORT, MOTOR_RF_IN1_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_RF_IN1_PORT, MOTOR_RF_IN2_PIN, Bit_RESET);
}

/**
  * @brief  右转（原地右旋）：左侧电机正转，右侧电机反转
  */
void Motor_Right(void)
{
    // 左侧电机正转
    GPIO_WriteBit(MOTOR_LF_IN1_PORT, MOTOR_LF_IN1_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_LF_IN1_PORT, MOTOR_LF_IN2_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_LB_IN1_PORT, MOTOR_LB_IN1_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_LB_IN1_PORT, MOTOR_LB_IN2_PIN, Bit_RESET);
    
    // 右侧电机反转
    GPIO_WriteBit(MOTOR_RB_IN1_PORT, MOTOR_RB_IN1_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_RB_IN1_PORT, MOTOR_RB_IN2_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_RF_IN1_PORT, MOTOR_RF_IN1_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_RF_IN1_PORT, MOTOR_RF_IN2_PIN, Bit_SET);
}

#endif


// 内部函数：设置单个电机的两个PWM通道
static void Motor_SetSingle(uint8_t id, int16_t speed) 
{
    const MotorConfig_t *m = &motor_cfg[id];

#if MOTOR_MODE_PWM

#if MOTOR_DERIVE_L298N

    uint32_t duty = 0;

    if (speed > 0) {
        // 正转
        GPIO_SetBits(m->gpio_in1, m->pin_in1);
        GPIO_ResetBits(m->gpio_in2, m->pin_in2);
        duty = (uint32_t)speed * PWM_MAX_DUTY / 100;
        //printf("1\r\n");
    } else if (speed < 0) {
        // 反转
        GPIO_ResetBits(m->gpio_in1, m->pin_in1);
        GPIO_SetBits(m->gpio_in2, m->pin_in2);
        duty = (uint32_t)(-speed) * PWM_MAX_DUTY / 100;
        //printf("2\r\n");
    } else {
        // 停止
        GPIO_ResetBits(m->gpio_in1, m->pin_in1);
        GPIO_ResetBits(m->gpio_in2, m->pin_in2);
        duty = 0;
        //printf("3\r\n");
    }

    // 设置 PWM 占空比
    if      (m->ch_pwm == 1) TIM_SetCompare1(m->tim, duty);
    else if (m->ch_pwm == 2) TIM_SetCompare2(m->tim, duty);
    else if (m->ch_pwm == 3) TIM_SetCompare3(m->tim, duty);
    else if (m->ch_pwm == 4) TIM_SetCompare4(m->tim, duty);

    //printf("ch_pwm:%d, duty:%d\r\n", m->ch_pwm, duty);

#else
    uint32_t duty_ina = 0;
	uint32_t duty_inb = 0;

    if (speed > 0) {
        duty_ina = (uint32_t)speed * PWM_MAX_DUTY / 100;
        duty_inb = 0;
    } else if (speed < 0) {
        duty_ina = 0;
        duty_inb = (uint32_t)(-speed) * PWM_MAX_DUTY / 100;
    } else {
        duty_ina = 0;
        duty_inb = 0;
    }
    // 设置PWM占空比（通过通道号设置比较值）
    // 使用宏简化：TIM_SetCompareX(TIMx, val)
    if (m->ch_ina == 1) TIM_SetCompare1(m->tim, duty_ina);
    else if (m->ch_ina == 2) TIM_SetCompare2(m->tim, duty_ina);
    else if (m->ch_ina == 3) TIM_SetCompare3(m->tim, duty_ina);
    else if (m->ch_ina == 4) TIM_SetCompare4(m->tim, duty_ina);

    if (m->ch_inb == 1) TIM_SetCompare1(m->tim, duty_inb);
    else if (m->ch_inb == 2) TIM_SetCompare2(m->tim, duty_inb);
    else if (m->ch_inb == 3) TIM_SetCompare3(m->tim, duty_inb);
    else if (m->ch_inb == 4) TIM_SetCompare4(m->tim, duty_inb);
#endif
	
	//printf("a:%d-d:%d;b:%d-d:%d\r\n", m->ch_ina, duty_ina, m->ch_inb, duty_inb);
#else
    // GPIO模式：仅控制方向（全速或停止）
    if (speed > 0) {
        GPIO_SetBits(m->gpio_ina, m->pin_ina);
        GPIO_ResetBits(m->gpio_inb, m->pin_inb);
    } else if (speed < 0) {
        GPIO_ResetBits(m->gpio_ina, m->pin_ina);
        GPIO_SetBits(m->gpio_inb, m->pin_inb);
    } else {
        GPIO_ResetBits(m->gpio_ina, m->pin_ina);
        GPIO_ResetBits(m->gpio_inb, m->pin_inb);
    }
#endif
}

/**
  * @brief  设置四个电机速度（带方向）
  * @param  lf: 左前轮速度 (-100~100)
  * @param  lb: 左后轮速度 (-100~100)
  * @param  rf: 右前轮速度 (-100~100)
  * @param  rb: 右后轮速度 (-100~100)
  */
void Motor_SetSpeed(int16_t lf, int16_t lb, int16_t rf, int16_t rb) 
{
    Motor_SetSingle(0, lf);   // 左前
    Motor_SetSingle(1, lb);   // 左后
    Motor_SetSingle(2, rf);   // 右前
    Motor_SetSingle(3, rb);   // 右后
}

// ---------- 逆运动学解算 ----------
// 参数 vx, vy, omega: 三个自由度的速度指令（单位任意，但映射到 -100~100 的轮速）
// 输出四个轮速，范围自动限幅到 -100~100
void Kinematics_Inverse(float vx, float vy, float omega, int16_t *lf, int16_t *lb, int16_t *rf, int16_t *rb) 
{
    // 麦轮安装参数：车轮到车体中心的距离（假设为1，实际比例可调节）
    const float Lx = 0.1f;   // 横向距离（宽度的一半）
    const float Ly = 0.1f;   // 纵向距离（长度的一半）
	float max_val;
    // 标准麦轮逆运动学公式（四轮独立驱动）
    float v_lf = vx - vy - omega * (Lx + Ly);
    float v_rf = vx + vy + omega * (Lx + Ly);
    float v_lb = vx + vy - omega * (Lx + Ly);
    float v_rb = vx - vy + omega * (Lx + Ly);

	
    // ---- 第一步：等比缩放，处理超过PWM_MAX的情况，保证方向不失真 ----
    max_val = fabs(v_lf);
    if (fabs(v_lb) > max_val) max_val = fabs(v_lb);
    if (fabs(v_rf) > max_val) max_val = fabs(v_rf);
    if (fabs(v_rb) > max_val) max_val = fabs(v_rb);

    if (max_val > PWM_MAX) {
        float scale = PWM_MAX / max_val;
        v_lf *= scale;
        v_lb *= scale;
        v_rf *= scale;
        v_rb *= scale;
    }
	
	    // ---- 第二步：低速死区重映射，处理低于PWM_MIN_EFFECTIVE的情况 ----
    *lf = remap_to_effective_range((int16_t)v_lf, PWM_MIN_EFFECTIVE, PWM_MAX);
    *lb = remap_to_effective_range((int16_t)v_lb, PWM_MIN_EFFECTIVE, PWM_MAX);
    *rf = remap_to_effective_range((int16_t)v_rf, PWM_MIN_EFFECTIVE, PWM_MAX);
    *rb = remap_to_effective_range((int16_t)v_rb, PWM_MIN_EFFECTIVE, PWM_MAX);
	
    // 归一化到 -100~100（取绝对值的最大值作为基准，防止溢出）
    // 这里假设 vx, vy, omega 已经是在 -100~100 范围内的标量，则直接限幅
//    *lf = LIMIT((int16_t)v_lf, -100, 100);
//    *lb = LIMIT((int16_t)v_lb, -100, 100);
//    *rf = LIMIT((int16_t)v_rf, -100, 100);
//    *rb = LIMIT((int16_t)v_rb, -100, 100);
}


// 仅计算左右两轮速度（同侧同速）
void Kinematics_Differential(float vx, float vy, int16_t *left, int16_t *right) 
{
    //const float L = 0.2f;   // 轮距，可调
    float l_speed = vx + vy;
    float r_speed = vx - vy;
    float max_val;

        // ---- 第一步：等比缩放，处理超过PWM_MAX的情况，保证方向不失真 ----
    max_val = fabs(l_speed);
    if (fabs(r_speed) > max_val) max_val = fabs(r_speed);

    if (max_val > PWM_MAX) {
        float scale = PWM_MAX / max_val;
        l_speed *= scale;
        r_speed *= scale;
    }
	
	    // ---- 第二步：低速死区重映射，处理低于PWM_MIN_EFFECTIVE的情况 ----
    *left = remap_to_effective_range((int16_t)l_speed, PWM_MIN_EFFECTIVE, PWM_MAX);
    *right = remap_to_effective_range((int16_t)r_speed, PWM_MIN_EFFECTIVE, PWM_MAX);
}

