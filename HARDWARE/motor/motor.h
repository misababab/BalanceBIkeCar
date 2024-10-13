#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

extern int PWM_MAX,PWM_MIN;
extern int MOTO1,MOTO2;

#define PWM_LED_G   TIM3->CCR1 //A6		PA6(G)°¢PA7(B)∏¯0¡¡
#define PWM_LED_B   TIM3->CCR2 //A7      PA5(R)	
#define PWMWHEEL   TIM3->CCR3 //B0	
#define PWMN20     TIM3->CCR4 //B1  	

#define EN   PBout(12) 
#define DR   PBout(13)  

void motor_GtrlPin_init(void);
void TIM4_Encoder_wheel_Init(u16 arr,u16 psc);
void TIM3_PWM_Motor_Led_Init(u16 arr,u16 psc);
int Read_Encoder(unsigned char TIMX);

void Limit(int *motoA);
void Load(int moto1);
void motor_n20_Disable(void);
void motor_n20_Forward(void);
void motor_n20_Back(void);


//RGN¡¡¬Ãµ∆
void openGreenLed(void);
//RGN¡¡¿∂µ∆
void openBlueLed(void);
//RGN¡¡∫Ïµ∆
void openRedLed(void);




#endif
