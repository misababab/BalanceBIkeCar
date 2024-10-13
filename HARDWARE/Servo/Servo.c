#include "Servo.h"
#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"
//高级定时器1pwm输出初始化
//arr：自动重装值（周期）  psc：时钟预分频数
void TIM1_PWM_Servo_Init(uint16_t arr, uint16_t psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 ,ENABLE);      
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO,ENABLE);

   //设置该引脚为复用输出功能,输出TIM1 CH1的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;//TIM_CH1  PA8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);  //使能TIM1在CCR1上的预装载寄存器	
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Disable);  //使能TIM1在CCR1上的预装载寄存器
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE 主输出使能	
	TIM_ARRPreloadConfig(TIM1,DISABLE);
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
}


 /* 自行车车头 */
//0.5ms--0°  2.5ms--180°CCR的范围：50-250
void servo_CtrlCarTrun(uint16_t angle)
{
  uint16_t pulse;
  
  //针对舵机可转角度限辐
  if(angle <= 5)
    angle = 5;
  if(angle >= 175)
    angle = 175;
  //将角度值转换为脉冲值  
  pulse = (uint16_t)(50 + 200 * angle/180.0);  //此转换公式需根据pwm的arr及psc配置来做相应变化
  TIM_SetCompare1(TIM1, pulse);
  
}

 /* 控制超声波摇头的舵机 */
void servo_CtrlWaveTrun(uint16_t angle)
{
  uint16_t pulse;
  
  //针对舵机可转角度限辐
  if(angle <= 0)
    angle = 0;
  if(angle >= 175)
    angle = 175;
  //将角度值转换为脉冲值  
  pulse = (uint16_t)(50 + 200 * angle/180.0);  //此转换公式需根据pwm的arr及psc配置来做相应变化
  TIM_SetCompare4(TIM1, pulse);
  
}


