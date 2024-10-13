#include "wave.h"
#include "delay.h"
#include "stm32f10x.h"
#include "usart2.h"
#define Trig GPIO_Pin_0
#define Echo GPIO_Pin_1

float Distance;
u8 overflow=0;
void TIM2_Update_Wave_Init(void)//Init_TIM1(9998,7199);
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructer;
	NVIC_InitTypeDef NVIC_InitStructer;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	/*��ʱ��TIM2��ʼ��*/
	TIM_DeInit(TIM2);

	TIM_TimeBaseInitStructer.TIM_Period=9999;//��ʱ����Ϊ10000
	TIM_TimeBaseInitStructer.TIM_Prescaler=719; //��Ƶϵ��720
	TIM_TimeBaseInitStructer.TIM_ClockDivision=TIM_CKD_DIV1;//����Ƶ
	TIM_TimeBaseInitStructer.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructer);

	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//���������ж�
	TIM_ITConfig(TIM2,TIM_IT_Trigger,ENABLE);
	TIM_Cmd(TIM2,DISABLE);//�رն�ʱ��ʹ��
	
	 // ��ʱ��2�ж�
    
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructer.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructer.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructer.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructer.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructer);
	
	
}
void TIM2_IRQHandler(void) //�жϣ��������źźܳ��ǣ�����ֵ������ظ����������ж��������������
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//����жϱ�־
		overflow=1;
	}
}


void Wave_CtrlPin_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitSture;
	EXTI_InitTypeDef  EXTI_InitSture;
	NVIC_InitTypeDef NVIC_InitStruct_extiA1;
	//����ⲿ�жϵĻ���һ��ʹ��AFIO���ù���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA,ENABLE);
	
	//����IO�˿�
	GPIO_InitSture.GPIO_Mode=GPIO_Mode_Out_PP;   //�������ģʽ
	GPIO_InitSture.GPIO_Pin=Trig;                //��PA0��Trig����
	GPIO_InitSture.GPIO_Speed=GPIO_Speed_50MHz;  
	GPIO_Init(GPIOA,&GPIO_InitSture);
	
	GPIO_InitSture.GPIO_Mode=GPIO_Mode_IPD;      //������ģʽ
	GPIO_InitSture.GPIO_Pin=Echo;                //��PA1��Echo����
	GPIO_InitSture.GPIO_Speed=GPIO_Speed_50MHz;  
	GPIO_Init(GPIOA,&GPIO_InitSture);
	
	//�жϺ�1�˿�ӳ��һ��
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);
	
	//�ⲿ�ж�����
	EXTI_InitSture.EXTI_Line=EXTI_Line1;
	EXTI_InitSture.EXTI_LineCmd=ENABLE;
	EXTI_InitSture.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitSture.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitSture);
	
	    //�ⲿ�ж�PA10
	NVIC_InitStruct_extiA1.NVIC_IRQChannel=EXTI1_IRQn;
	NVIC_InitStruct_extiA1.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct_extiA1.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStruct_extiA1.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStruct_extiA1);
	
}

void EXTI1_IRQHandler(void)
{   
	delay_us(10);	
	if(EXTI_GetITStatus(EXTI_Line1)!=RESET)
	{
		TIM_SetCounter(TIM2,0);
		TIM_Cmd(TIM2,ENABLE);
		while(GPIO_ReadInputDataBit(GPIOA,Echo)&&overflow==0)  //�ȴ��͵�ƽ	
		{
			if(TIM_GetCounter(TIM2)>800){break;}//�������136cmmʱ,�˳�ѭ��//136*2/340=0.8s
		}
		TIM_Cmd(TIM2,DISABLE);
		//TIM_GetCounter(TIM2)�Ƕ�Ԥ��Ƶ��ļ���ʱ�ӽ��м�����		
		//����ʱ��ÿ��һ�������ؼ�������ֵ��+1��������Ҳ��16λ�ģ������ֵ���Դ�0~65535
		
		Distance=(float)(TIM_GetCounter(TIM2)*340)/2000;	
		if(TIM_GetCounter(TIM2)<2){Distance=1;}		
		if(overflow==1)
		{
			Distance=136;
			overflow=0;
		}
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void Wave_Strat(void)
{
	GPIO_SetBits(GPIOA,Trig);   //��Trig����Ϊ�ߵ�ƽ
	delay_us(20);               //��������10us����������������ģ�鹤��
	GPIO_ResetBits(GPIOA,Trig); 	
}

