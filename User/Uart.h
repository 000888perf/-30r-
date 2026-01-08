#include "stm32f10x.h"                  // Device header

#ifndef _Uart_H_
#define _Uart_H_
#include "string.h"
#include "stdarg.h"
// 2. 声明所有需要在main.c中使用的变量（extern关键字）
extern uint8_t Serial_TxBuffer[128]; 
#define SERIAL_RX_BUFFER_SIZE 128//这个宏定义也要帮.c声明
extern uint8_t Serial_RxBuffer[128];   // DMA接收缓冲区
extern uint8_t Serial_RxPacket[128];   // 处理用数据包缓存
extern uint16_t Serial_RxLength;                         // 接收长度
extern uint8_t Serial_RxFlag;                            // 接收完成标志



void Serial_DMA_RX_Init(void);
void Serial_DMA_TX_Init(void);
void DMA_USART1_Init(void);
void USART1_CTS_Init(void);

void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array,uint16_t Length);
void Serial_SendString(char*String);
void Serial_Printf(char *format,...);

void USART1_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);


#endif



