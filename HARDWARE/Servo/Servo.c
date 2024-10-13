#include "Servo.h"
#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"
//�߼���ʱ��1pwm�����ʼ��
//arr���Զ���װֵ�����ڣ�  psc��ʱ��Ԥ��Ƶ��
void TIM1_PWM_Servo_Init(uint16_t arr, uint16_t psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 ,ENABLE);      
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO,ENABLE);

   //���ø�����Ϊ�����������,���TIM1 CH1��PWM���岨��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;//TIM_CH1  PA8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 80K
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���	
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Disable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE �����ʹ��	
	TIM_ARRPreloadConfig(TIM1,DISABLE);
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
}


 /* ���г���ͷ */
//0.5ms--0��  2.5ms--180��CCR�ķ�Χ��50-250
void servo_CtrlCarTrun(uint16_t angle)
{
  uint16_t pulse;
  
  //��Զ����ת�Ƕ��޷�
  if(angle <= 5)
    angle = 5;
  if(angle >= 175)
    angle = 175;
  //���Ƕ�ֵת��Ϊ����ֵ  
  pulse = (uint16_t)(50 + 200 * angle/180.0);  //��ת����ʽ�����pwm��arr��psc����������Ӧ�仯
  TIM_SetCompare1(TIM1, pulse);
  
}

 /* ���Ƴ�����ҡͷ�Ķ�� */
void servo_CtrlWaveTrun(uint16_t angle)
{
  uint16_t pulse;
  
  //��Զ����ת�Ƕ��޷�
  if(angle <= 0)
    angle = 0;
  if(angle >= 175)
    angle = 175;
  //���Ƕ�ֵת��Ϊ����ֵ  
  pulse = (uint16_t)(50 + 200 * angle/180.0);  //��ת����ʽ�����pwm��arr��psc����������Ӧ�仯
  TIM_SetCompare4(TIM1, pulse);
  
}


