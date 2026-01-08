#include "./stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Uart.h"
#include "ADC.h"
int main(void)
	{
		//初始化串口
		USART1_CTS_Init();
		//初始化DMA	
		Serial_DMA_RX_Init();
		Serial_DMA_TX_Init();
		
		ADC_InitSturucture();
		while (1)
			{
				Delay_ms(1000);			
				Serial_Printf("%d\r\n",ADC1_Data);
			}
	}