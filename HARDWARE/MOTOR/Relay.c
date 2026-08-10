#include "stm32f10x.h"
#include "Relay.h"
#include "OLED.h"
#include "Delay.h"
#include "PWM.h"

void Motor_Init(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET);
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_RESET);
   
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_7, Bit_RESET);

}


void Motor_dao( u8 turn ){
    if(turn == 1){
        GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET);
    }
    else{
        GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET);
    }
}

void relay_water(u8 turn){
    if(turn == 1){
        GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_SET);
    }
    else{
        GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_RESET);
    }
}


/*
    GPIOA, GPIO_Pin_0   in3
    GPIOA, GPIO_Pin_1   in1
    GPIOA, GPIO_Pin_2   in2
    GPIOA, GPIO_Pin_3   in4
*/

/**
  * @brief  行走电机复位
  * @param  无
  * @retval 无
  */

void Motor_init(void){
    GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);
}

/**
  * @brief  升降电机复位
  * @param  无
  * @retval 无
  */

void Spray_init(void){
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_7, Bit_RESET);
}

     
/**
  * @brief  左电机制动
  * @param  trun     口子向外
            trun 0   后
            trun 1   前
  * @retval 无
  */
void Motor_Left(u8 trun){
    if(trun == 1){
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
        
        
    }
    else{
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
    }
}

/**
  * @brief  右电机制动
  * @param  trun     口子向外
            trun 0   后
            trun 1   前
  * @retval 无
  */

void Motor_Rigth(u8 trun){
    if(trun == 1){
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);
    }
    else{
       //右后
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_SET);
    }
}

//电机逆时针
void Motor_Counterclockwise(void){
    Motor_Rigth(0);
    Motor_Left(0);
}
//电机顺时针
void Motor_Clockwise(void){
    Motor_Rigth(0);
    Motor_Left(1);
}

/**
  * @brief  右升降电机
  * @param  trun 0   下
            trun 1   上
  * @retval 无
  */
void Rigth_Spray(u8 trun){
    if(trun == 1){
        //右上
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
    }
    else{
    ////右下
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
    }
}

/**
  * @brief  左升降电机
  * @param  trun 0   下
            trun 1   上
  * @retval 无
  */

void Left_Tape(u8 trun){
    if(trun == 1){
////左上   
        GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_SET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_7, Bit_RESET);

    }
    else{
//    //左下   
        GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_7, Bit_SET);
    }
}

/**
  * @brief  剪刀舵机复位，切下，复位
  * @param  无
  * @retval 无
  */

void Scissors_servo(void){
    PWM_SetCompare3(2000);//打开
    delay_ms(100);
    PWM_SetCompare3(1000);//剪断开
    delay_ms(800);
    PWM_SetCompare3(2000);
}

/**
  * @brief  阻挡舵机拦和放
  * @param  trun
             1      拦
             0      放
  * @retval 无
  */     

void Embrace_servo(u8 trun){
    if(trun == 1){
        PWM_SetCompare2(1500);
        PWM_SetCompare4(1800);
    }
    else{
        PWM_SetCompare2(900);
        PWM_SetCompare4(2500);
    }
}

///**
//  * @brief  胶带舵机伸出收回
//  * @param  trun
//             1      往外
//             0      往回
//  * @retval 无
//  */

void  Tape_servo(u8 trun){
    if(trun == 1){
//        for(int i = 500 ; i <= 2500 ;  ){
            PWM_SetCompare1(700);
//            Delay_ms(50);
//            i += 100;
//        }
    }
    else{
//        for(int i = 500 ; i <= 1800 ;  ){
            PWM_SetCompare1(1800);
//            Delay_ms(50);
//            i += 100;
//        }    
    }
}

extern u8 Spray_up,Spray_Down,Tape_up,Tape_Down;
extern u8 timer_Sign,timer_Sign1;
//0,顺时针转动 1,前进 2,后退 3,停止
void Pwm_Motor(u8 turn , u16 PWM){
    switch(turn){
        case 0: PWM1_SetTime2(0);//顺时针转动
                PWM2_SetTime2(0);
                PWM3_SetTime2(PWM);
                PWM4_SetTime2(PWM);break;
        
        case 1: PWM1_SetTime2(PWM);//前进
                PWM2_SetTime2(0);
                PWM3_SetTime2(PWM);
                PWM4_SetTime2(0);break;
        
        case 2: PWM1_SetTime2(0);//后退
                PWM2_SetTime2(PWM);
                PWM3_SetTime2(0);
                PWM4_SetTime2(PWM);break;
        
        case 3:PWM1_SetTime2(PWM);//停止
                PWM2_SetTime2(0);
                PWM3_SetTime2(0);
                PWM4_SetTime2(PWM);break;
    }

}

void Run(void){
    //初始化 两侧电机到底,胶带复位
    if(1){
        Tape_servo(1);//胶带收回
        Embrace_servo(0);
        Motor_dao(0);
        relay_water(0);
        Delay_ms(100);
        
        while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) != 0){
            Rigth_Spray(0);//
        }
        Spray_init();
        while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) != 0){
            Left_Tape(1);
        }
        Spray_init();
        }
    
    if(1){//前行检测树
//        Motor_Left(1);
//        Motor_Rigth(1);
        //0,顺时针转动 1,前进 2,后退 3,停止
        PWM1_SetTime2(6000);//前进
        PWM2_SetTime2(0);
        PWM3_SetTime2(6000);
        PWM4_SetTime2(0);
        
        while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) != 0);
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
//        Motor_init();//两侧电机停止
        //0,顺时针转动 1,前进 2,后退 3,停止
        Pwm_Motor(3 , 3500);
        
        Embrace_servo(1);//检测到树后拦住  
        Tape_servo(0);//胶带伸出     
        Delay_ms(100);  
    }
    
    //缠胶舵机贴树,剪断胶带
    if(1){
//        Tape_servo(1);//伸出胶带侧舵机
//        Delay_ms(500);
//        Tape_servo(0);
        
       //0,顺时针转动 1,前进 2,后退 3,停止
        Pwm_Motor(0 , 3500);
//        Motor_Clockwise();//顺时针旋转
        Delay_ms(20000);//先缠胶带5s
//        Motor_init();//两侧电机停止                //0,顺时针转动 1,前进 2,后退 3,停止
        Pwm_Motor(3 , 3500);
        Motor_dao(1);
        Delay_ms(5000);
        Motor_dao(0);
        Tape_servo(1);//胶带收回
//        Scissors_servo();//剪刀侧舵机进行裁剪
//        Tape_servo(0);//收回胶带侧舵机
    } 

    
    //喷水上升并且喷水，车呈现旋转状态
    if(1){
        relay_water(1);
        Rigth_Spray(1);//边转别喷2s
        Delay_ms(5000);
//        Motor_Clockwise();
        //0,顺时针转动 1,前进 2,后退 3,停止
        Pwm_Motor(0 , 3500);
        Delay_ms(5000);
        Spray_init();
        Delay_ms(15000);

    }
    
    //车停
    if(1){
        relay_water(0);
        Spray_init();
//        Motor_init();
        //0,顺时针转动 1,前进 2,后退 3,停止
        Pwm_Motor(3 , 3500);
        Delay_ms(100);
    }
    
    //上升胶带到顶端,缠胶带3s
    if(1){
        Left_Tape(0);
        while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) != 0);
        Spray_init();
        Tape_servo(0);//胶带伸出
//        Tape_servo(0);
//        Delay_ms(500);
//        Tape_servo(0);
        //0,顺时针转动 1,前进 2,后退 3,停止
        Pwm_Motor(0 , 3500);
//        Motor_Clockwise();
        Delay_ms(20000);//缠胶带3s
//        Scissors_servo();
//        Tape_servo(0);
        //0,顺时针转动 1,前进 2,后退 3,停止
        Pwm_Motor(3 , 3500);
//        Motor_init();
        Motor_dao(1);
        Delay_ms(5000);
        Motor_dao(0);
        Delay_ms(10);
        Tape_servo(1);//胶带收回
    }
    if(1){
        Embrace_servo(0);

//        Motor_Left(0);
//        Motor_Rigth(0);
        //0,顺时针转动 1,前进 2,后退 3,停止
        Pwm_Motor(2 , 5000);
        Delay_ms(1200);
//        Motor_init();
        Pwm_Motor(3 , 3500);
    }
    

}
