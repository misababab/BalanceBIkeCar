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
	
	/*定时器TIM2初始化*/
	TIM_DeInit(TIM2);

	TIM_TimeBaseInitStructer.TIM_Period=9999;//定时周期为10000
	TIM_TimeBaseInitStructer.TIM_Prescaler=719; //分频系数720
	TIM_TimeBaseInitStructer.TIM_ClockDivision=TIM_CKD_DIV1;//不分频
	TIM_TimeBaseInitStructer.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructer);

	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//开启更新中断
	TIM_ITConfig(TIM2,TIM_IT_Trigger,ENABLE);
	TIM_Cmd(TIM2,DISABLE);//关闭定时器使能
	
	 // 定时器2中断
    
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructer.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructer.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructer.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructer.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructer);
	
	
}
void TIM2_IRQHandler(void) //中断，当回响信号很长是，计数值溢出后重复计数，用中断来保存溢出次数
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//清除中断标志
		overflow=1;
	}
}


void Wave_CtrlPin_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitSture;
	EXTI_InitTypeDef  EXTI_InitSture;
	NVIC_InitTypeDef NVIC_InitStruct_extiA1;
	//如果外部中断的话则一定使能AFIO复用功能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA,ENABLE);
	
	//配置IO端口
	GPIO_InitSture.GPIO_Mode=GPIO_Mode_Out_PP;   //推挽输出模式
	GPIO_InitSture.GPIO_Pin=Trig;                //将PA0于Trig相连
	GPIO_InitSture.GPIO_Speed=GPIO_Speed_50MHz;  
	GPIO_Init(GPIOA,&GPIO_InitSture);
	
	GPIO_InitSture.GPIO_Mode=GPIO_Mode_IPD;      //拉输入模式
	GPIO_InitSture.GPIO_Pin=Echo;                //将PA1于Echo相连
	GPIO_InitSture.GPIO_Speed=GPIO_Speed_50MHz;  
	GPIO_Init(GPIOA,&GPIO_InitSture);
	
	//中断和1端口映射一起
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);
	
	//外部中断配置
	EXTI_InitSture.EXTI_Line=EXTI_Line1;
	EXTI_InitSture.EXTI_LineCmd=ENABLE;
	EXTI_InitSture.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitSture.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitSture);
	
	    //外部中断PA10
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
		while(GPIO_ReadInputDataBit(GPIOA,Echo)&&overflow==0)  //等待低电平	
		{
			if(TIM_GetCounter(TIM2)>800){break;}//距离大于136cmm时,退出循环//136*2/340=0.8s
		}
		TIM_Cmd(TIM2,DISABLE);
		//TIM_GetCounter(TIM2)是对预分频后的计数时钟进行计数，		
		//计数时钟每来一个上升沿计数器的值就+1，计数器也是16位的，里面的值可以从0~65535
		
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
	GPIO_SetBits(GPIOA,Trig);   //将Trig设置为高电平
	delay_us(20);               //持续大于10us触发，触发超声波模块工作
	GPIO_ResetBits(GPIOA,Trig); 	
}

