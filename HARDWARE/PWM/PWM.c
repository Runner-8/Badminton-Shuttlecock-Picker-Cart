#include "PWM.h"

// 舵机参数
#define PULSE_MIN       500
#define PULSE_MAX       2500
#define ANGLE_MAX_H     270.0f
#define ANGLE_MAX_V     180.0f
#define PULSE_PER_DEG_H ((PULSE_MAX - PULSE_MIN) / ANGLE_MAX_H)  // 7.407
#define PULSE_PER_DEG_V ((PULSE_MAX - PULSE_MIN) / ANGLE_MAX_V)  // 11.111

// 全局变量（存储当前角度）
float current_angle_h = 135.0f;
float current_angle_v = 125.0f;

// ===================== PWM初始化 =====================
void PWM_Init(u32 arr, u32 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // PA6 -> CH1 (垂直), PA7 -> CH2 (水平)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_InternalClockConfig(TIM3);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  // PA6
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  // PA7

    TIM_Cmd(TIM3, ENABLE);
}

// ===================== 角度设置函数 =====================
// 水平：PA7 (TIM3_CH2)
void Servo_SetAngle_H(float angle_deg)
{

    uint16_t pulse;


    if(angle_deg<0)

        angle_deg=0;


    if(angle_deg>270)

        angle_deg=270;


    pulse=
    500+
    angle_deg*7.407;



    TIM_SetCompare2(
    TIM3,
    pulse);


    current_angle_h=angle_deg;

}


// 垂直：PA6 (TIM3_CH1)
void Servo_SetAngle_V(float angle_deg)
{


    uint16_t pulse;


    if(angle_deg<0)

        angle_deg=0;



    if(angle_deg>180)

        angle_deg=180;



    pulse=
    500+
    angle_deg*11.11;



    TIM_SetCompare1(
    TIM3,
    pulse);



    current_angle_v=angle_deg;


}


// ===================== 初始化入口 =====================
void Servo_Init(void)
{
    PWM_Init(20000, 72);     // 20ms周期，50Hz（72MHz时钟）
    Servo_SetAngle_H(135.0f);
    Servo_SetAngle_V(125.0f);
}
