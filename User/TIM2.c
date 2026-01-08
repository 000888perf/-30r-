#include "stm32f10x.h"                  // Device header
#include "TIM2.h"
#include "Delay.h"
#include "Uart.h"
#include "ADC.h"
//--------------------------------------------------------//
uint32_t LED_PEM = 0;//设置初始LED占空比为0
uint32_t MODE_1 = 0;//状态机
uint32_t MODE_2 = 0;//状态机
uint32_t MODE_3 = 0;//状态机
uint32_t MODE_4 = 0;//状态机
uint8_t LED_time = 100;//设置灯亮度转换间隔#这个没有用了(又有用了)
//--------------------------------------------------------//

void LED_InitStructure(void)
{
		//配置引脚
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);//打开时钟
	
	GPIO_InitTypeDef GPIO_Initstructure;//定义结构体
	/*GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_PP;			//GPIOA1引脚做输出引脚	
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;//选择引脚
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_Initstructure);//把结构体数据传入GPIO引脚函数*/
	
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF_PP;//GPIOA2引脚做复用输出引脚
	GPIO_Initstructure.GPIO_Pin =GPIO_Pin_2;//选择引脚pwm输出引脚
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_Initstructure);//把结构体数据传入GPIO引脚函数
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//打开复用引脚必须打开复用时钟
//-------------------------------------------------------------------------------------------------	
	//打开TIM2时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//TIM2使用的AP1的是时钟
	
	//配置定时器基本参数 当前配置定时
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//采样时钟分频，设置一秒内的采样时钟，越高精度越高
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up	;//向上计数
	TIM_TimeBaseStructure.TIM_Period = 100-1;//pwm周期设置为100us
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;//预分频器
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	

	
	//配置NVIC中断（定义TIM2的通断优先级，和打开中断服务）
	NVIC_InitTypeDef NVIC_InitStructue;
	NVIC_InitStructue.NVIC_IRQChannel = TIM2_IRQn;//用的TIM2定时器的中断通道
	NVIC_InitStructue.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructue.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructue.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructue);

	//让打开定时器的溢出，会申请中断
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);	
	
	//配置pwm模式
	// 配置PWM模式（通道3）配置的原理是把pwm比较控制器的寄存器写入改变
	TIM_OCInitTypeDef TIM_OCStruct;//定义结构体
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;//控制当前pwm模式为1
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;//控制是否要输出，现为输出模式
	TIM_OCStruct.TIM_Pulse =1-1;//占空比 = (TIM_Pulse / (ARR+1)) × 100%
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;//控制输出高电平还是低电平，现为高电平（是控制pwm波形，占空比中是输出高还是低电平，波形翻转）
	TIM_OC3Init(TIM2, &TIM_OCStruct);//选择tm2的通道3（PGIOA2）

//TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);//预装载TIM2的占空比，避免占空比突变#LED都一样不用还没有延迟呢
   
	 //TIM_ARRPreloadConfig(TIM2, ENABLE); // 预装载ARR,等当前周期结束再输入新的周期，这个对TIM2都生效#不改变频率就不加了
	
	// 3. 配置CH4比较模式（核心：使能CH4比较中断）#就是个定时中断触发器，靠总线的时钟实现计数
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;   // 定时模式（仅匹配中断，无输出）
	// 若需PWM输出，改为TIM_OCMode_PWM1/TIM_OCMode_PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; // 禁用输出（仅中断）
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      // 极性（输出时有效）
	TIM_OCInitStructure.TIM_Pulse =100-1;//timing纯定时器模式，这个是触发计数的时间，一个tim2的周期只触发一次
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);      
  TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);// 使能CH4比较中断	
	
		// 3. 配置CH2比较模式（核心：使能CH2比较中断）
	TIM_OCInitTypeDef TIM_OCInitStructure_1;
	TIM_OCInitStructure_1.TIM_OCMode = TIM_OCMode_Timing;   // 定时模式（仅匹配中断，无输出）
	// 若需PWM输出，改为TIM_OCMode_PWM1/TIM_OCMode_PWM2
	TIM_OCInitStructure_1.TIM_OutputState = TIM_OutputState_Disable; // 禁用输出（仅中断）
	TIM_OCInitStructure_1.TIM_OCPolarity = TIM_OCPolarity_High;      // 极性（输出时有效）
	TIM_OCInitStructure_1.TIM_Pulse =100-1;//timing纯定时器模式，这个是触发计数的时间，一个tim2的周期只触发一次
	TIM_OC2Init(TIM2, &TIM_OCInitStructure_1);      
  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);// 使能CH2比较中断	
	
	//启动定时器
	TIM_Cmd(TIM2,ENABLE);//TIM_Cmd定时器控制器，ENABLE= 开启定时器，DISABLE= 关闭定时器；
	
}


void LED_ON(void)
{
	for (LED_PEM=0;LED_PEM<=99;LED_PEM++)
		{

			TIM_SetCompare3(TIM2, LED_PEM);//把LED占空比写入
		}
	LED_PEM=0;
}

void LED_OFF(void)
{
	for (LED_PEM=0;LED_PEM<=99;LED_PEM++)
		{

			TIM_SetCompare3(TIM2, 99-LED_PEM);//把LED占空比写入
		}
	LED_PEM=0;
}
		
//中断函数的优先级高于主程序
void TIM2_IRQHandler(void)//定时器的中断函数,函数名是固定
		//TIM2：指定处理 TIM2 的中断，IRQ：Interrupt Request（中断请求）， Handler：处理程序；
		//不同的中断都需要单独的if和清除，有更新中断和比较中断
{
		if (TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)//检查中断标志位
		{
			TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  // 清除Update中断标志(一定要清除发出中断的定时器)
			MODE_2++;
			if (MODE_2>2500)
				{	
					MODE_2=0;
					//功能删除了
				}	
		}
	
		if (TIM_GetITStatus(TIM2,TIM_IT_CC4) != RESET)//检查中断标志位
		{
			TIM_ClearITPendingBit(TIM2,TIM_IT_CC4);  // 清除CH4中断标志(一定要清除发出中断的定时器)
			MODE_1++;
			if (MODE_1>5000)
				{	
					MODE_1=0;

					}	
		}

		
		if (TIM_GetITStatus(TIM2,TIM_IT_CC2) != RESET)//检查中断标志位
		{
			TIM_ClearITPendingBit(TIM2,TIM_IT_CC2);  // 清除CH2中断标志(一定要清除发出中断的定时器)
			MODE_4++;
			if (MODE_4>LED_time)
				{
					LED_PEM++;
					MODE_4 = 0;
					FSM_Process();
				}
		}
			
}

void FSM_Process(void)
{
	switch(MODE_3)
		{
			case 0: 
				TIM_SetCompare3(TIM2, LED_PEM);//把LED占空比写入
				if (LED_PEM>=99)
					{
						LED_PEM = 0;
						MODE_3 = 1;
						
					}
					break;
			case 1: 
				TIM_SetCompare3(TIM2, 100-LED_PEM);//把LED占空比写入
				if (LED_PEM>=99)
					{
						LED_PEM = 0;
						MODE_3 = 0;
						
					}
				break;
		}



}
