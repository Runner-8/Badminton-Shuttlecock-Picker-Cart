#include "PWM.h"

// ===================== PWM初始化 =====================
void PWM_Init(u32 arr, u32 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // PA0 -> CH1 , PA1 -> CH2 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_InternalClockConfig(TIM2);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);  // PA0
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);  // PA1

    TIM_Cmd(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

}

/**
 * @brief 设置风扇转速（输入百分比 0~100）
 * @param ch 通道FAN_CH1/FAN_CH2
 * @param percent 占空比百分比 0~100
 */
void Fan_SetSpeed(FanChannel_t ch, uint8_t percent)
{
    uint16_t ccrVal;
    // 限幅，防止超出0~100范围
    if(percent > 100)
        percent = 100;
    
    // 百分比换算为CCR寄存器值：percent/100 * PWM_MAX_CCR
    ccrVal = (uint16_t)((uint32_t)percent * PWM_MAX_CCR / 100U);

    switch(ch)
    {
        case FAN_CH1:
            TIM_SetCompare1(TIM2, ccrVal);
            break;
        case FAN_CH2:
            TIM_SetCompare2(TIM2, ccrVal);
            break;
        default: break;
    }
}

// ===================== 初始化入口 =====================
void Fan_Init(void)
{
    PWM_Init(40, 72);    
    Fan_SetSpeed(FAN_CH1, 0);
    Fan_SetSpeed(FAN_CH2, 0);
}
