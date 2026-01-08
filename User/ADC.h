#include "stm32f10x.h"                  // Device header

#ifndef _ADC_H_
#define _ADC_H_

extern uint16_t ADC1_Data;
extern float ADC1_numerical_value;//转化电压的，代码还没写

void ADC_InitSturucture(void);
void DMA1_Channel1_IRQHandler(void);

#endif
