#include "stm32f10x.h"
#ifndef __SERVO_H
#define __SERVO_H



// 舵机角度控制函数声明
void Servo_Init(void);        // 舵机初始化（PWM输出）
void Servo_SetAngle(u8 angle);// 设置舵机角度（0-180°）

extern uint16_t cmp_val_1;

#endif
