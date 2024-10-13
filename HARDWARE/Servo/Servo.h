#ifndef __DUOJI_H
#define __DUOJI_H
#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"

void TIM1_PWM_Servo_Init(uint16_t arr, uint16_t psc);
void servo_CtrlCarTrun(uint16_t angle);
void servo_CtrlWaveTrun(uint16_t angle);
#endif
