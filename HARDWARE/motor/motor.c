#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"
#include "usart.h"
#include "PID.h"

/*  ��ʱ����Ϊ���֣�PWM��ʱ������������ʱ���������ж϶�ʱ��
    �����֣�  ��������       PB12   ->   EN
             ת����������    PB13   ->   DR
             PWM�����ź�     PB0��TIM3-ch3��
             ������AB��      PB6     ->   TIM4-ch1
                             PB7     ->   TIM4-ch2

    N20����� ��������      PC14
                            PC15
              PWM�����ź�   PB1��TIM3-ch4��

    RGB������ PWM�����ź�   PA6(G)��TIM3-ch1����PA7(B)��TIM3-ch2��
                һ������            PA5(R)
*/

int PWM_MAX=7199, PWM_MIN=-7199;
int MOTO1;

void motor_GtrlPin_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;           /* RGB��R�� */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;    /*wheel�����������*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);         /* �͵�ƽ ��ʼ��������Ϊֹͣ */

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13|GPIO_Pin_14 | GPIO_Pin_15; /* N20��������������  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* tim4��Ϊ�����ֵı�������ʱ�� */
void TIM4_Encoder_wheel_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    //ʹ�ܶ�ʱ��3ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //ʹ��GPIO�����AFIO���ù���ģ��ʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;   //�˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);                        //�����趨������ʼ��GPIOB
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;   //�˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);                        //�����趨������ʼ��GPIOB
    //��ʼ��TIM4
    TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
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

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  //ʹ�ܶ�ʱ��3ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE); //ʹ��GPIOB��ʱ��

    GPIOA_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;          //�������
    GPIOA_InitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;                //PA7��PA6    ����������
    GPIOA_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIOA_InitStruct);

    GPIOB_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;          //�������
    GPIOB_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;                //PB0��PB1    n20����Ͷ����ֵ�PWM�����ź�
    GPIOB_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIOB_InitStruct);

    TIM_TimeBaseInitStruct.TIM_Period = arr;              //�趨�������Զ���װֵ
    TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          //�趨Ԥ��Ƶ��
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;         //����ʱ�ӷָ�
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);      //��ʼ����ʱ��

    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��PWM1ģʽ
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
    TIM_OCInitStruct.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
    /* ����������Ч��ƽΪ�͵�ƽ����Ϊ���չ˶�������ˢ����ģ���ˢ�����PWM�����ź�ʱ�͵�ƽ��Ч */
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OC1Init(TIM3, &TIM_OCInitStruct);                      //��ʼ������Ƚϲ���
    TIM_OC2Init(TIM3, &TIM_OCInitStruct);                      //��ʼ������Ƚϲ���
    TIM_OC3Init(TIM3, &TIM_OCInitStruct);                      //��ʼ������Ƚϲ���
    TIM_OC4Init(TIM3, &TIM_OCInitStruct);                      //��ʼ������Ƚϲ���
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH2ʹ��Ԥװ�ؼĴ���
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH2ʹ��Ԥװ�ؼĴ���
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH2ʹ��Ԥװ�ؼĴ���
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH2ʹ��Ԥװ�ؼĴ���
    TIM_ARRPreloadConfig(TIM3, ENABLE);                //ʹ��TIM1��ARR�ϵ�Ԥװ�ؼĴ���

    TIM_Cmd(TIM3, ENABLE);                             //ʹ�ܶ�ʱ��3
}

/* ��ȡ�����ֵı�������ֵ */
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
    if (TIM4->SR&0X0001) //����ж�
    {
    }

    TIM4->SR&=~(1<<0);//����жϱ�־λ
}

/*�޷�����*/
void Limit(int *motoA)
{
    if (*motoA>PWM_MAX)*motoA=PWM_MAX;

    if (*motoA<PWM_MIN)*motoA=PWM_MIN;
}

/*����ֵ����*/
int GFP_abs(int p)
{
    int q;
    q=p>=0?p:(-p);
    return q;
}

/*��ֵ����*/
/*��ڲ�����PID������ɺ������PWMֵ*/
void Load(int moto1)//moto1=-200����ת200������
{
    //1.�о������ţ���Ӧ����ת
    if (moto1>=0)        EN=1, DR=1; //��ת

    if (moto1<0)         EN=1, DR=0; //��ת

    //2.�о�PWMֵ
    //TIM_SetCompare3(TIM3,GFP_abs(moto1));//PWMA
    PWMWHEEL=GFP_abs(moto1);
}

/* �ر�n20��� */
void motor_n20_Disable(void)
{
    PCout(14)=0;
    PCout(15)=0;
}

/* n20���ǰ�� */
void motor_n20_Forward(void)
{
    PCout(14)=0;
    PCout(15)=1;
}

/* n20������� */
void motor_n20_Back(void)
{
    PCout(14)=1;
    PCout(15)=0;
}

//RGN���̵�
void openGreenLed(void)
{
    PAout(5)=1;
    PWM_LED_B=0;
    PWM_LED_G=7199;
}

//RGN������
void openBlueLed(void)
{
    PAout(5)=1;
    PWM_LED_B=7199;
    PWM_LED_G=0;
}

//RGN�����
void openRedLed(void)
{
    PAout(5) = 0;//�����
    PWM_LED_B = 0;
    PWM_LED_G = 0;

}
