#include "./stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Uart.h"
#include "ADC.h"
#include "servo.h"
#include "TIM2.h"
int main(void)
	{
		//初始化串口
		USART1_CTS_Init();
		//初始化DMA	
		Serial_DMA_RX_Init();
		Serial_DMA_TX_Init();
		//初始化ADC
		ADC_InitSturucture();
		//初始化PWM
		Servo_Init();        // 舵机初始化（PWM输出）
		
		LED_InitStructure();


		
		uint32_t time = 500;//延迟
		while (1)
			{	
				Delay_ms(20);
				Serial_Printf("次数=%d,ADC数值=%d\r\n",i,ADC1_Data);
				i++;
	

				Delay_ms(time);
				Servo_SetAngle(0);// 设置舵机角度（0-180°）
				Serial_Printf("次数=%d，舵机角度=%d\r\n",i,cmp_val_1);//不给printf加间隔会导致发的内容乱码
				i++;
				
				Delay_ms(time);
				Servo_SetAngle(45);// 设置舵机角度（0-180°）
				Serial_Printf("次数=%d，舵机角度=%d\r\n",i,cmp_val_1);
				i++;
			}
	}
