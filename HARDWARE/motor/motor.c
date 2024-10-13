#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"
#include "usart.h"
#include "PID.h"

/*  定时器分为三种：PWM定时器、编码器定时器、更新中断定时器
    动量轮：  控制引脚       PB12   ->   EN
             转动方向引脚    PB13   ->   DR
             PWM驱动信号     PB0（TIM3-ch3）
             编码器AB相      PB6     ->   TIM4-ch1
                             PB7     ->   TIM4-ch2

    N20电机： 控制引脚      PC14
                            PC15
              PWM驱动信号   PB1（TIM3-ch4）

    RGB呼吸灯 PWM驱动信号   PA6(G)（TIM3-ch1）、PA7(B)（TIM3-ch2）
                一般驱动            PA5(R)
*/

int PWM_MAX=7199, PWM_MIN=-7199;
int MOTO1;

void motor_GtrlPin_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;           /* RGB的R灯 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;    /*wheel方向控制引脚*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);         /* 低电平 初始化动量轮为停止 */

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13|GPIO_Pin_14 | GPIO_Pin_15; /* N20电机方向控制引脚  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* tim4作为动量轮的编码器定时器 */
void TIM4_Encoder_wheel_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    //使能定时器3时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //使能GPIO外设和AFIO复用功能模块时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;   //端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);                        //根据设定参数初始化GPIOB
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;   //端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);                        //根据设定参数初始化GPIOB
    //初始化TIM4
    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    //Reset counter
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

void TIM3_PWM_Motor_Led_Init(u16 arr, u16 psc)
{
    //////////////////////
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;
    GPIO_InitTypeDef GPIOA_InitStruct;
    GPIO_InitTypeDef GPIOB_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  //使能定时器3时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOB的时钟

    GPIOA_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;          //复用输出
    GPIOA_InitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;                //PA7、PA6    两个呼吸灯
    GPIOA_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIOA_InitStruct);

    GPIOB_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;          //复用输出
    GPIOB_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;                //PB0、PB1    n20电机和动量轮的PWM驱动信号
    GPIOB_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIOB_InitStruct);

    TIM_TimeBaseInitStruct.TIM_Period = arr;              //设定计数器自动重装值
    TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          //设定预分频器
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;         //设置时钟分割
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);      //初始化定时器

    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //选择PWM1模式
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStruct.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
    /* 这里设置有效电平为低电平，是为了照顾动量轮无刷电机的，无刷电机的PWM驱动信号时低电平有效 */
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OC1Init(TIM3, &TIM_OCInitStruct);                      //初始化输出比较参数
    TIM_OC2Init(TIM3, &TIM_OCInitStruct);                      //初始化输出比较参数
    TIM_OC3Init(TIM3, &TIM_OCInitStruct);                      //初始化输出比较参数
    TIM_OC4Init(TIM3, &TIM_OCInitStruct);                      //初始化输出比较参数
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH2使能预装载寄存器
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH2使能预装载寄存器
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH2使能预装载寄存器
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH2使能预装载寄存器
    TIM_ARRPreloadConfig(TIM3, ENABLE);                //使能TIM1在ARR上的预装载寄存器

    TIM_Cmd(TIM3, ENABLE);                             //使能定时器3
}

/* 读取动量轮的编码器数值 */
int Read_Encoder(unsigned char TIMX)
{
    int Encoder_TIM;

    switch (TIMX)
    {
        case 4:
            Encoder_TIM= (short)TIM4 -> CNT;
            TIM4 -> CNT=0;
            break;

        default:
            Encoder_TIM=0;
    }

    return Encoder_TIM;
}

void TIM4_IRQHandler(void)
{
    if (TIM4->SR&0X0001) //溢出中断
    {
    }

    TIM4->SR&=~(1<<0);//清除中断标志位
}

/*限幅函数*/
void Limit(int *motoA)
{
    if (*motoA>PWM_MAX)*motoA=PWM_MAX;

    if (*motoA<PWM_MIN)*motoA=PWM_MIN;
}

/*绝对值函数*/
int GFP_abs(int p)
{
    int q;
    q=p>=0?p:(-p);
    return q;
}

/*赋值函数*/
/*入口参数：PID运算完成后的最终PWM值*/
void Load(int moto1)//moto1=-200：反转200个脉冲
{
    //1.研究正负号，对应正反转
    if (moto1>=0)        EN=1, DR=1; //正转

    if (moto1<0)         EN=1, DR=0; //反转

    //2.研究PWM值
    //TIM_SetCompare3(TIM3,GFP_abs(moto1));//PWMA
    PWMWHEEL=GFP_abs(moto1);
}

/* 关闭n20电机 */
void motor_n20_Disable(void)
{
    PCout(14)=0;
    PCout(15)=0;
}

/* n20电机前进 */
void motor_n20_Forward(void)
{
    PCout(14)=0;
    PCout(15)=1;
}

/* n20电机后退 */
void motor_n20_Back(void)
{
    PCout(14)=1;
    PCout(15)=0;
}

//RGN亮绿灯
void openGreenLed(void)
{
    PAout(5)=1;
    PWM_LED_B=0;
    PWM_LED_G=7199;
}

//RGN亮蓝灯
void openBlueLed(void)
{
    PAout(5)=1;
    PWM_LED_B=7199;
    PWM_LED_G=0;
}

//RGN亮红灯
void openRedLed(void)
{
    PAout(5) = 0;//亮红灯
    PWM_LED_B = 0;
    PWM_LED_G = 0;

}
