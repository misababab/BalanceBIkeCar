#include "usart2.h"
#include "usart.h"
#include "delay.h"
#include "stdio.h"
#include <stdarg.h>
#include "math.h"
#include "PID.h"
#include <stdlib.h>
#include <string.h>
#include "Servo.h"
///////////////////////
//pid���Ʋ���

u8 usart2_flag=100;
u8 usart2_CacheByte;//����2Cache


void usart2_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;// ����һ��GPIO_InitTypeDef���͵ı���
    USART_InitTypeDef USART_InitStructure;// ����һ��USART_InitTypeDef���͵ı���
    NVIC_InitTypeDef NVIC_InitStructure;

    // ����ʹ��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    USART_DeInit(USART2);  //��λ����2 -> ����û��

    // ��ʼ�� ���ڶ�ӦIO��  TX-PA2  RX-PA3
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ��ʼ�� ����ģʽ״̬
    USART_InitStructure.USART_BaudRate=bound; // ������
    /*****************/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;// 8������λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 1��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No ; // ����żУ��
    /*****************/
    USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None; // Ӳ��������
    USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx; // ���� ���� ģʽ��ʹ��
    USART_InitStructure.USART_Parity=USART_Parity_No; // û����żУ��
    USART_InitStructure.USART_StopBits=USART_StopBits_1; // һλֹͣλ
    USART_InitStructure.USART_WordLength=USART_WordLength_8b; // ÿ�η������ݿ��Ϊ8λ
    USART_Init(USART2, &USART_InitStructure);

    /*   ʹ�ܴ���   */
    USART_Cmd(USART2, ENABLE);
    /* �򿪿����ж� */
    //USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    /* �򿪽����ж� */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    // ��ʼ�� �ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel=USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
    NVIC_Init(&NVIC_InitStructure);

}


void printf2(char *fmt, ...)
{
    char buffer[CMD_BUFFER_LEN+1];
    u8 i = 0;
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    vsnprintf(buffer, CMD_BUFFER_LEN+1, fmt, arg_ptr);

    while ((i < CMD_BUFFER_LEN) && buffer[i])
    {
        USART_SendData(USART2, (u8) buffer[i++]);

        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    }
}


void USART2_IRQHandler(void) // ����2�жϷ�����
{
    u8 res;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET) // �жϱ�־
    {
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);   //USART1
        res= USART_ReceiveData(USART2);  // ����2 ����
       
		usart2_CacheByte=res;/* ����Cache */

        if (res==0x00)		/* ����usart2_flag��־λ */
        {
            usart2_flag=0;//ͣ
        }
        else if (res==0x01)
        {
            usart2_flag=1;//ǰ��
        }
        else if (res==0x02)
        {
            usart2_flag=2;//����
        }
        else if (res==0x03)
        {
            usart2_flag=3;//����
        }
        else if (res==0x04)
        {
            usart2_flag=4;//��ת30��
        }
        else if (res==0x05)
        {
            usart2_flag=5;//��ת30��
        }
        else if (res==0x06)
        {
            usart2_flag=6;//ң��ģʽ
        }
        else if (res==0x07)
        {
            usart2_flag=7;//����ģʽ
        }
        else if (res==0x0a)
        {
            usart2_flag=8;
        }
        else if (res==0x0b)
        {
            usart2_flag=9;
        }
        else if (res==0x0c)
        {
            usart2_flag=10;
        }
        else if (res==0x0d)//�س�\r
        {
            usart2_flag=11;
        }
        else
        {
            usart2_flag=100;
        }

    }
}


/* ���������ݷ��� */
/*//uint8_t aRxBuffer2[20]= {0x00};
//uint8_t RxCounter2=0;
//uint8_t ReceiveState2=0;
//uint8_t i2=0;

//uint8_t c2=0;

//int x[10], t;

//int bufA2[20]= {0x00}, bufB2[20]= {0x00}, bufC2[20]= {0x00}, bufD2[20]= {0x00}, bufE2[20]= {0x00}, bufF2[20]= {0x00}, bufG2[20]= {0x00}, bufH2[20]= {0x00}, bufI2[20]= {0x00}, bufJ2[20]= {0x00};

//int buffsize2=0;
//u8 t1A2=0;
//u8 t1B2=0;
//u8 t1C2=0, t1D2=0, t1E2=0, t1F2=0, t1G2=0, t1H2=0, t1I2=0, t1J2=0;

//u8 flag2=0;
//int numA2=0;
//int numB2=0, numC2=0, numD2=0, numE2=0, numF2=0, numG2=0, numH2=0, numI2=0, numJ2=0;

//int read_A2=0, read_B2=0, read_C2=0, read_D2=0, read_E2=0, read_F2=0, read_G2=0, read_H2=0, read_I2=0, read_J2=0;

int j2=0;*/



/**********************************
���ڷ�������
**********************************/
//����Ŀû�õ�
void Usart2_send(u8 com)
{
    USART_ClearFlag(USART2, USART_FLAG_TC);
    USART_SendData(USART2, com);

    while (!USART_GetFlagStatus(USART2, USART_FLAG_TC));
}


//����Ŀû�õ�
void uart2_SendData(u8 ch)
{
    USART_SendData(USART2, ch);

    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE)==0);
}

//����Ŀû�õ�
void uart2_SendString(u8 *ch)
{
    while (*ch!='\0')
    {
        USART_SendData(USART2, *ch);

        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE)==0);

        ch++;
    }
}


/////////////////////////////////////////////////


/*
void USART2_IRQHandler(void)
{
    uint8_t Clear=Clear;//���ֶ��巽��������������������"û���õ�"����
//  char receive;
    //static char res=0;

    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) // ������յ�1���ֽ�
    {
        aRxBuffer2[RxCounter2++] = USART2->DR;// �ѽ��յ����ֽڱ��棬�����ַ��1
        if(USART2->DR=='0')
        {
            usart2_flag=0;
        }
        if(USART2->DR=='1')//ǰ��
        {
            usart2_flag=1;
        }
        if(USART2->DR==0x03)//ǰ��
        {
            usart2_flag=2;
        }

    }
//  else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)// ������յ�1֡����
//  {
//      Clear=USART2->SR;// ��SR�Ĵ���
//      Clear=USART2->DR;// ��DR�Ĵ���(�ȶ�SR�ٶ�DR������Ϊ�����IDLE�ж�)
//      ReceiveState2=1;// ��ǽ��յ���1֡����
//  }
    //if(res)usart2receive();
}
*/


//void usart2receive(void)
//{
//    if (ReceiveState2==1) //������յ�1֡����
//    {
//        ReceiveState2=0;
//        i2=0;

//        while (RxCounter2--) // �ѽ��յ����ݷ��ͻش���
//        {
//            c2=aRxBuffer2[i2++];

//            if (c2=='$')
//            {
//                TIM_Cmd(TIM6, DISABLE);
//                TIM_Cmd(TIM8, DISABLE);
//            }
//            else if (c2=='!')
//            {
//                TIM_Cmd(TIM6, ENABLE);
//                TIM_Cmd(TIM8, ENABLE);
//            }


//            if (c2=='A')
//            {
//                flag2=1;
//            }

//            if (flag2==1&&(c2>='0'&& c2<='9')) //if(c>='0'&& c<='9')
//            {
//                numA2=c2 - '0';

//                bufA2[t1A2]=numA2;
//                t1A2++;
//            }


//            if (c2=='B')
//            {
//                flag2=2;
//            }

//            if (flag2==2&&(c2>='0'&& c2<='9')) //if(c>='0'&& c<='9')
//            {
//                numB2=c2 - '0';

//                bufB2[t1B2]=numB2;
//                t1B2++;
//            }

//            if (c2=='C')
//            {
//                flag2=3;
//            }

//            if (flag2==3&&(c2>='0'&& c2<='9')) //if(c>='0'&& c<='9')
//            {
//                numC2=c2 - '0';

//                bufC2[t1C2]=numC2;
//                t1C2++;
//            }

//            if (c2=='D')
//            {
//                flag2=4;
//            }

//            if (flag2==4&&(c2>='0'&& c2<='9')) //if(c>='0'&& c<='9')
//            {
//                numD2=c2 - '0';

//                bufD2[t1D2]=numD2;
//                t1D2++;
//            }

//            if (c2=='E')
//            {
//                flag2=5;
//            }

//            if (flag2==5&&(c2>='0'&& c2<='9')) //if(c>='0'&& c<='9')
//            {
//                numE2=c2 - '0';

//                bufE2[t1E2]=numE2;
//                t1E2++;
//            }

//            if (c2=='F')
//            {
//                flag2=6;
//            }

//            if (flag2==6&&(c2>='0'&& c2<='9')) //if(c>='0'&& c<='9')
//            {
//                numF2=c2 - '0';

//                bufF2[t1F2]=numF2;
//                t1F2++;
//            }

//            if (c2=='G')
//            {
//                flag2=7;
//            }

//            if (flag2==7&&(c2>='0'&& c2<='9')) //if(c>='0'&& c<='9')
//            {
//                numG2=c2 - '0';

//                bufG2[t1G2]=numG2;
//                t1G2++;
//            }

//            if (c2=='H')
//            {
//                flag2=8;
//            }

//            if (flag2==8&&(c2>='0'&& c2<='9')) //if(c>='0'&& c<='9')
//            {
//                numH2=c2 - '0';

//                bufH2[t1H2]=numH2;
//                t1H2++;
//            }

//            if (c2=='I')
//            {
//                flag2=9;
//            }

//            if (flag2==9&&(c2>='0'&& c2<='9')) //if(c>='0'&& c<='9')
//            {
//                numI2=c2 - '0';

//                bufI2[t1I2]=numI2;
//                t1I2++;
//            }

//            if (c2=='J')
//            {
//                flag2=10;
//            }

//            if (flag2==10&&(c2>='0'&& c2<='9')) //if(c>='0'&& c<='9')
//            {
//                numJ2=c2 - '0';

//                bufJ2[t1J2]=numJ2;
//                t1J2++;
//            }
//        }

//        RxCounter2=0;

//        for (j2=0; j2<t1A2; j2++)
//        {
//            read_A2=read_A2 + bufA2[j2]*pow(10, (t1A2-j2-1));
//            x[0]=read_A2;
//        }

//        for (j2=0; j2<t1B2; j2++)
//        {
//            read_B2=read_B2 + bufB2[j2]*pow(10, (t1B2-j2-1));
//            x[1]=read_B2;
//        }


//        for (j2=0; j2<t1C2; j2++)
//        {
//            read_C2=read_C2 + bufC2[j2]*pow(10, (t1C2-j2-1));
//            x[2]=read_C2;
//        }

//        for (j2=0; j2<t1D2; j2++)
//        {
//            read_D2=read_D2 + bufD2[j2]*pow(10, (t1D2-j2-1));
//            x[3]=read_D2;
//        }

//        for (j2=0; j2<t1E2; j2++)
//        {
//            read_E2=read_E2 + bufE2[j2]*pow(10, (t1E2-j2-1));
//            x[4]=read_E2;
//        }

//        for (j2=0; j2<t1F2; j2++)
//        {
//            read_F2=read_F2 + bufF2[j2]*pow(10, (t1F2-j2-1));
//            x[5]=read_F2;
//        }

//        for (j2=0; j2<t1G2; j2++)
//        {
//            read_G2=read_G2 + bufG2[j2]*pow(10, (t1G2-j2-1));
//            x[6]=read_G2;
//        }

//        for (j2=0; j2<t1H2; j2++)
//        {
//            read_H2=read_H2 + bufH2[j2]*pow(10, (t1H2-j2-1));
//            x[7]=read_H2;
//        }

//        for (j2=0; j2<t1I2; j2++)
//        {
//            read_I2=read_I2 + bufI2[j2]*pow(10, (t1I2-j2-1));
//            x[8]=read_I2;
//        }

//        for (j2=0; j2<t1J2; j2++)
//        {
//            read_J2=read_J2 + bufJ2[j2]*pow(10, (t1J2-j2-1));
//            x[9]=read_J2;
//        }



//        for (t=0; t<10; t++)
//        {
//            //printf2("  %d  ",x[t]);
//        }



//        //A566 B234 C345 D321 E5678 F432 G970 H651 I21 J90

//        //printf2("PID(%d , %d, %d, %d, %d, %d, %d, %d, %d, %d)\n",Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Balance_Kp1,Balance_Kd1,Velocity_Kp1,Velocity_Ki1,Turn_Kp,Turn_Kd);

//        t1A2 = 0;
//        t1B2 = 0;
//        t1C2 = 0;
//        t1D2 = 0;
//        t1E2 = 0;
//        t1F2 = 0;
//        t1G2 = 0;
//        t1H2 = 0;
//        t1I2 = 0;
//        t1J2 = 0;


//        read_A2 = 0;
//        read_B2 = 0;
//        read_C2 = 0;
//        read_D2 = 0;
//        read_E2 = 0;
//        read_F2 = 0;
//        read_G2 = 0;
//        read_H2 = 0;
//        read_I2 = 0;
//        read_J2 = 0;

//        flag2 = 0;
//        memset(bufA2, 0, 20);
//        memset(bufB2, 0, 20);
//        memset(bufC2, 0, 20);
//        memset(bufD2, 0, 20);
//        memset(bufE2, 0, 20);
//        memset(bufF2, 0, 20);
//        memset(bufG2, 0, 20);
//        memset(bufH2, 0, 20);
//        memset(bufI2, 0, 20);
//        memset(bufJ2, 0, 20);
//        memset(aRxBuffer2, 0, 20);
//        //          TIM_ITConfig(TIM8,TIM8_UP_IRQn,1);
//        //          TIM_ITConfig(TIM6,TIM6_IRQn,1);
//        //          TIM_Cmd(TIM6,1);
//        //        TIM_Cmd(TIM8,1);
//    }
//}





