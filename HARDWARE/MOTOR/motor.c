#include "stm32f10x.h"                  // Device header
#include "motor.h"
#include "bluetooth.h"

void LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
	GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
}

void ir_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*
    Motor control pins:
    左边两轮电机驱动:  左前：PB12, PB13, 左后：PB14, PB15
    右边两轮电机驱动:  右后：PB5, PB6, 右前：PB7, PB8
*/
#define LEFT_FRONT_IN1  GPIO_Pin_12
#define LEFT_FRONT_IN2  GPIO_Pin_13
#define LEFT_REAR_IN1   GPIO_Pin_14
#define LEFT_REAR_IN2   GPIO_Pin_15

#define RIGHT_FRONT_IN1 GPIO_Pin_5
#define RIGHT_FRONT_IN2 GPIO_Pin_6
#define RIGHT_REAR_IN1  GPIO_Pin_7
#define RIGHT_REAR_IN2  GPIO_Pin_8

// 全部电机引脚掩码（用于初始化）
#define ALL_MOTOR_PINS  (GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | \
                         GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15)

/**
  * @brief  初始化电机控制引脚为推挽输出
  */
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Pin = ALL_MOTOR_PINS;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 默认全部停止
    Motor_Stop();
}

/**
  * @brief  所有电机停止
  */
void Motor_Stop(void)
{
    // 左侧两个电机
    GPIO_WriteBit(GPIOB, LEFT_FRONT_IN1, Bit_RESET);
    GPIO_WriteBit(GPIOB, LEFT_FRONT_IN2, Bit_RESET);
    GPIO_WriteBit(GPIOB, LEFT_REAR_IN1, Bit_RESET);
    GPIO_WriteBit(GPIOB, LEFT_REAR_IN2, Bit_RESET);
    
    // 右侧两个电机
    GPIO_WriteBit(GPIOB, RIGHT_FRONT_IN1, Bit_RESET);
    GPIO_WriteBit(GPIOB, RIGHT_FRONT_IN2, Bit_RESET);
    GPIO_WriteBit(GPIOB, RIGHT_REAR_IN1, Bit_RESET);
    GPIO_WriteBit(GPIOB, RIGHT_REAR_IN2, Bit_RESET);
}

/**
  * @brief  前进：所有电机正转
  * @note   正转定义为 IN1=1, IN2=0（可根据实际接线调整）
  */
void Motor_Forward(void)
{
    // 左侧两个电机正转
    GPIO_WriteBit(GPIOB, LEFT_FRONT_IN1, Bit_SET);
    GPIO_WriteBit(GPIOB, LEFT_FRONT_IN2, Bit_RESET);
    GPIO_WriteBit(GPIOB, LEFT_REAR_IN1, Bit_SET);
    GPIO_WriteBit(GPIOB, LEFT_REAR_IN2, Bit_RESET);
    
    // 右侧两个电机正转
    GPIO_WriteBit(GPIOB, RIGHT_FRONT_IN1, Bit_SET);
    GPIO_WriteBit(GPIOB, RIGHT_FRONT_IN2, Bit_RESET);
    GPIO_WriteBit(GPIOB, RIGHT_REAR_IN1, Bit_SET);
    GPIO_WriteBit(GPIOB, RIGHT_REAR_IN2, Bit_RESET);
}

/**
  * @brief  后退：所有电机反转
  * @note   反转为 IN1=0, IN2=1
  */
void Motor_Backward(void)
{
    // 左侧两个电机反转
    GPIO_WriteBit(GPIOB, LEFT_FRONT_IN1, Bit_RESET);
    GPIO_WriteBit(GPIOB, LEFT_FRONT_IN2, Bit_SET);
    GPIO_WriteBit(GPIOB, LEFT_REAR_IN1, Bit_RESET);
    GPIO_WriteBit(GPIOB, LEFT_REAR_IN2, Bit_SET);
    
    // 右侧两个电机反转
    GPIO_WriteBit(GPIOB, RIGHT_FRONT_IN1, Bit_RESET);
    GPIO_WriteBit(GPIOB, RIGHT_FRONT_IN2, Bit_SET);
    GPIO_WriteBit(GPIOB, RIGHT_REAR_IN1, Bit_RESET);
    GPIO_WriteBit(GPIOB, RIGHT_REAR_IN2, Bit_SET);
}

/**
  * @brief  左转（原地左旋）：左侧电机反转，右侧电机正转
  */
void Motor_Left(void)
{
    // 左侧电机反转
    GPIO_WriteBit(GPIOB, LEFT_FRONT_IN1, Bit_RESET);
    GPIO_WriteBit(GPIOB, LEFT_FRONT_IN2, Bit_SET);
    GPIO_WriteBit(GPIOB, LEFT_REAR_IN1, Bit_RESET);
    GPIO_WriteBit(GPIOB, LEFT_REAR_IN2, Bit_SET);
    
    // 右侧电机正转
    GPIO_WriteBit(GPIOB, RIGHT_FRONT_IN1, Bit_SET);
    GPIO_WriteBit(GPIOB, RIGHT_FRONT_IN2, Bit_RESET);
    GPIO_WriteBit(GPIOB, RIGHT_REAR_IN1, Bit_SET);
    GPIO_WriteBit(GPIOB, RIGHT_REAR_IN2, Bit_RESET);
}

/**
  * @brief  右转（原地右旋）：左侧电机正转，右侧电机反转
  */
void Motor_Right(void)
{
    // 左侧电机正转
    GPIO_WriteBit(GPIOB, LEFT_FRONT_IN1, Bit_SET);
    GPIO_WriteBit(GPIOB, LEFT_FRONT_IN2, Bit_RESET);
    GPIO_WriteBit(GPIOB, LEFT_REAR_IN1, Bit_SET);
    GPIO_WriteBit(GPIOB, LEFT_REAR_IN2, Bit_RESET);
    
    // 右侧电机反转
    GPIO_WriteBit(GPIOB, RIGHT_FRONT_IN1, Bit_RESET);
    GPIO_WriteBit(GPIOB, RIGHT_FRONT_IN2, Bit_SET);
    GPIO_WriteBit(GPIOB, RIGHT_REAR_IN1, Bit_RESET);
    GPIO_WriteBit(GPIOB, RIGHT_REAR_IN2, Bit_SET);
}
