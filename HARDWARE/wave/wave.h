#ifndef __WAVE_H
#define __WAVE_H	
#include "sys.h"

extern float Distance;

void gpio_csb_init(void);
void time2_csb_init(void);
void nvic_csb_init(void);
u16 dist_sensor(void);

void TIM2_Update_Wave_Init(void);
void Wave_CtrlPin_Init(void);
void EXTI1_IRQHandler(void);
void Wave_Strat(void);
#endif
