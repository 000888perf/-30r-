#include "servo.h"

uint16_t cmp_val =0;
uint16_t cmp_val_1 =0;

// TIM1_CH1(PA8) PWM输出初始化
void Servo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);
    
    // PA8 推挽复用输出配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	
    // TIM1时基配置（72MHz主频，输出50Hz PWM）
    TIM_TimeBaseStructure.TIM_Period = 20000-1;      // 自动重装值（周期=20ms）
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;        // 预分频器（72MHz/72=1MHz）
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;     // 时钟分割
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
    // PWM模式配置（通道1）
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 使能输出
    TIM_OCInitStructure.TIM_Pulse = 500; // 初始占空比（1.5ms，对应90°）
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 高电平有效
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    
    // 使能通道1预装载
    //TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    // 使能TIM1主输出（必须！否则无PWM输出）
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    // 使能TIM1
    TIM_Cmd(TIM1, ENABLE);
		
		//开启ARR预装载
		//TIM_ARRPreloadConfig(TIM1,ENABLE);//虽然用不到但不影响开不开一样
		
		 //开启CCR1预装载
    //TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);用这个有神秘bug,开了很影响舵机速度
}

// 设置舵机角度（0-180°）
void Servo_SetAngle(u8 angle)
{

    // 角度范围限制（防止超出舵机量程）
    if(angle > 180) angle = 180;

    cmp_val_1 = angle;
    // 计算比较值：0.5ms(500) ~ 2.5ms(2500) 对应 0°~180°
    // 公式：cmp_val =  ≈ 500 + angle * 11.11
    cmp_val =500+(((uint32_t)angle*2000)/180) ;
		
    // 设置比较值（更新PWM高电平时间）
    TIM_SetCompare1(TIM1, cmp_val);
		cmp_val =0;
}
