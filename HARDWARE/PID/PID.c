#include "stm32f10x.h"
#include "sys.h"
#include "stdlib.h"
#include "stdio.h"
#include "usart.h"
#include "usart2.h"
#include "motor.h"
#include "PID.h"
#include "Servo.h"
#include "adc.h"
#include "systick.h"
#include "mpu6050.h"
#include "mpuiic.h"
#include "delay.h"
#include "wave.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

u8 count1=0;
int count2=0;
int count3=0;
int count4=0;

int Encoder_Wheel;          //�з��ű��������������
u8 bizhang_mode=0;

u8 turn_stop_flag=0;
//u8 turn_left_flag=0;
//u8 turn_right_flag=0;
u8 delay1=0;
u8 delay2=0;
u8 delay3=0;
u8 delay4=0;

float left_distance=0;
float right_distance=0;

float Med_Angle=0;//��е��ֵ-��ʹ��С������ƽ��ס�ĽǶȡ�
float
	Angle_Kp=-1280,	
	Angle_Ki=0,
	Angle_Kd=-4.5;
float
	Velocity_Kp=60,	
	Velocity_Ki=0.24;
//float
//	Angle_Kp=-1280,	
//	Angle_Ki=0,
//	Angle_Kd=-8.5;
//float
//	Velocity_Kp=60,	
//	Velocity_Ki=0.24;
int Vertical_PWM,Velocity_PWM;//ֱ����&�ٶȻ����������

void control(void)//10msִ��һ��
{
	int PWM_out;
	/* ��ȡ�ٶ� */
	Encoder_Wheel = -Read_Encoder(4);  //�ӱ������ж�ȡ��ǰ���ӵ��ٶȣ���λ�ã�����ȡ���Եõ�ʵ�ʵ��ٶ�ֵ��  
	/* ��ȡ�Ƕȡ����ٶ�  */
	mpu_dmp_get_data(&pitch,&roll,&yaw);			//�Ƕ�
	
	if(roll>=0) {PCout(13)=1;}
	if(roll<0)  {PCout(13)=0;}
	
	/* PIDƽ����� */
	Vertical_PWM = Vertical_PD(Med_Angle,roll,gyro[0]); /* ֱ���� */
	Velocity_PWM = Velocity_PI(Encoder_Wheel);		    /* �ٶȻ� */	
	PWM_out = Vertical_PWM + Velocity_PWM;				/* ������� */
	
	if(roll<-15||roll>15)//�жϳ�����
	{
		PWM_out = Velocity_IncCtrl_PI(Encoder_Wheel,0);	//�������ٶ�ǿ����Ϊ��
		count3 = -2;//�ر�rgb����ɫ�仯
		openRedLed();//�����
	}		
	Limit(&PWM_out);	
	Load(PWM_out);//���ص�����ϡ�
	
	/* 10ms +1 */
	count1++;
	count2++; 
	count3++;
	count4++;

	//ÿ50ms��ȡһ�δ���ֵ
	if(count1==5)
	{
		count1=0;
		
		if(usart2_CacheByte=='y'|| usart2_CacheByte=='b')//�ж�ң�������ͣ�100��������λ����stm32ң������
		{
			usart2_flag=100;
		}
		if(usart2_CacheByte=='b'|| usart2_flag==7)//����ģʽ
		{ 
			bizhang_mode=1; 
		}
		if(usart2_CacheByte=='y'|| usart2_flag==6)//ң��ģʽ
		{ 
			bizhang_mode=0;  
			usart2_CacheByte='0'; //����ң��ģʽ��ͣ���ȴ��û�
		}
		
		if(bizhang_mode==0)//ң��ģʽ
		{
			/********����ģʽ�ı�������*******/
			turn_stop_flag=0;
			delay1=0;
			delay2=0;
			delay3=0;
			delay4=0;
			/*********************************/
			/* ����ǰ��OR���� */
			if(usart2_CacheByte=='0'|usart2_flag==0){motor_n20_Disable();}//ͣ
			if(usart2_CacheByte=='1'|usart2_flag==1){motor_n20_Forward();}//ǰ��
			if(usart2_CacheByte=='2'|usart2_flag==2){motor_n20_Back();}//����
			/* �ٶ� */
			if(usart2_CacheByte=='A'){TIM_SetCompare4(TIM3,7199);}				 //С���ٶ�0����ͣ��
			if(usart2_CacheByte=='B'|usart2_flag==8){TIM_SetCompare4(TIM3,4000);}//С���ٶ�1��
			if(usart2_CacheByte=='C'|usart2_flag==9){TIM_SetCompare4(TIM3,2500);}//С���ٶ�2��
			if(usart2_CacheByte=='D'|usart2_flag==10){TIM_SetCompare4(TIM3,1000);}//С���ٶ�3��
			if(usart2_CacheByte=='E'|usart2_flag==11){TIM_SetCompare4(TIM3,1);}//С���ٶ�4������죩
			/* ��ͷ���� */
			if(usart2_CacheByte=='F'){servo_CtrlCarTrun(86-50);}//��ת45��
			if(usart2_CacheByte=='G'){servo_CtrlCarTrun(86-44);}//��ת40��
			if(usart2_CacheByte=='H'){servo_CtrlCarTrun(86-38);}//��ת35��
			if(usart2_CacheByte=='I'){servo_CtrlCarTrun(86-32);}//��ת30��
			if(usart2_CacheByte=='J'|usart2_flag==4){servo_CtrlCarTrun(86-27);}//��ת25��
			if(usart2_CacheByte=='K'){servo_CtrlCarTrun(86-22);}//��ת20��
			if(usart2_CacheByte=='L'){servo_CtrlCarTrun(86-16);}//��ת15��
			if(usart2_CacheByte=='M'){servo_CtrlCarTrun(86-11);}//��ת10��
			if(usart2_CacheByte=='N'){servo_CtrlCarTrun(86-6);}//��ת5��
			
			if(usart2_CacheByte=='O'|usart2_flag==3){servo_CtrlCarTrun(86);}  //��ͷ����
			
			if(usart2_CacheByte=='P'){servo_CtrlCarTrun(86+6);} //��ת5��
			if(usart2_CacheByte=='Q'){servo_CtrlCarTrun(86+11);}//��ת10��
			if(usart2_CacheByte=='R'){servo_CtrlCarTrun(86+16);}//��ת15��
			if(usart2_CacheByte=='S'){servo_CtrlCarTrun(86+22);}//��ת20��
			if(usart2_CacheByte=='T'|usart2_flag==5){servo_CtrlCarTrun(86+27);}//��ת25��
			if(usart2_CacheByte=='U'){servo_CtrlCarTrun(86+32);}//��ת30��
			if(usart2_CacheByte=='V'){servo_CtrlCarTrun(86+38);}//��ת35��
			if(usart2_CacheByte=='W'){servo_CtrlCarTrun(86+44);}//��ת40��
			if(usart2_CacheByte=='X'){servo_CtrlCarTrun(86+50);}//��ת45��
		}
		if(bizhang_mode==1)//����ģʽ
		{		
			TIM_SetCompare4(TIM3,2500);//�ٶȶ������б���
			/* ǰ�����ϰ� */
			if(Distance>25&&turn_stop_flag==0)
			{
				PCout(14)=0;PCout(15)=1;	//ǰ��
				count3=-2;					//�ر�rgb����ɫ�仯
				openGreenLed();				//���̵�
			}
			/* �����ϰ��� */
			if(Distance<=25&turn_stop_flag==0)
			{
				PCout(14)=0;PCout(15)=0;	//ͣ
				count3=-2;					//�ر�rgb����ɫ�仯
				openBlueLed();				//������
				
				turn_stop_flag=1;
			}
			if(turn_stop_flag==1)
			{
				delay1++;//50ms����
				/* ״̬1 ���ͷ������� */
				/* delay1״̬(delay1<10)++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
				if(delay1<10)			   			//delay1��ʱ����ʱ0-500ms��������ϰ���ʱ��	  
				{
					servo_CtrlWaveTrun(165);	 	/* Wave���ͷ */
				}
				if(delay1==10)
				{
					left_distance = Distance;		/* ��ȡWave���ͷ���� */
					servo_CtrlWaveTrun(0);		 	/* Wave�Ұ�ͷ */
				}
				/* ״̬2 �Ұ�ͷ������� */
				/* delay2״̬ (delay2<=10 && delay1>10)++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
				if(delay1>=10){delay2++;delay1=11;} //delay2��ʱ����ʱ0-500ms��������ϰ���ʱ��
				if(delay2==10)	
				{
					right_distance = Distance; 	 	/* ��ȡWave�Ұ�ͷ����*/
					servo_CtrlWaveTrun(81);		 	/* ����Wave */
				}
				/* ״̬3 ת����� */			
				/* delay3״̬ (delay3<=5 && delay2>10)+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
				if(delay2>=10){delay3++;delay2=11;}	//delay3��ʱ����ʱ0-250ms		
				if(delay3==5)//�����жϳ�ͷ��������
				{
					if(left_distance>=right_distance && left_distance>30)/* ������ϰ�����ͷ���� */
					{
						servo_CtrlCarTrun(86-39);//��ת40��
					}
					
					if(left_distance<right_distance && right_distance>30)/* �ұ����ϰ�����ͷ���� */
					{
						servo_CtrlCarTrun(86+40);//��ת40�� 
					}
					
					if(left_distance<=30 && right_distance<=30)/* ���Ҷ����ϰ����ͷ��ֱ������Ŀ�ĵ�ͣ�� */
					{
						motor_n20_Disable();//ͣ
						servo_CtrlCarTrun(86);//������ͷ
						count3 = -2;	//�ر�rgb����ɫ�仯
						openRedLed();//�����
			
						delay3 = 0;/* ��Delay3���㲻���к���������״̬����3 */
					}
				}
				/* ״̬4 ��������ת��һ�ξ��� */
				/* delay4״̬ (delay4<=35 && delay3>5)+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
				if(delay3>=5){delay4++;delay3=6;}//delay4��ʱ����ʱ1750ms	
				if(delay3>=5 && delay4<35)
				{
					PCout(14)=0;PCout(15)=1;//ǰ��
				}
				if(delay4==35)
				{
					motor_n20_Disable();		//ͣ
					servo_CtrlCarTrun(86);		//��ͷ����
					turn_stop_flag=0;
					delay1=0;
					delay2=0;
					delay3=0;
					delay4=0;
				}				
			}		
		}					
	}
	//10ms
	if(usart2_flag!=100)//stm32ң����ң�ء�С���ϴ����ݸ�ң����
	{
		if(count2==100)//ÿ1000���봮�ڷ��͵�ص�ѹAD����
		{			
			count2=0;
			
			printf2("t16.txt=\"%.1f\"",AdcValue); 
			printf2("\xff\xff\xff");	//��β������������Ч����	
		}
		if(count4==50)//500ms����һ�νǶȺ;�����Ϣ
		{
			count4=0;
			
			Wave_Strat();
			printf2("t17.txt=\"%.1f\"",roll); 
			printf2("\xff\xff\xff");
			printf2("t18.txt=\"%.1f\"",Distance); 
			printf2("\xff\xff\xff");
		}
	}
	//10ms
	if(usart2_flag==100)//������λ��ң��С��
	{	
		if(count2%2==0){printf2("r%d",(int)(roll*100.0));}//ÿ20���봮�ڷ���һ�β�����			
		if(count2==3){printf2("v%d",(int)(AdcValue*100.0));}//100ms���ڵĵ�30ms
		if(count2==5){printf2("d%d",(int)(Distance*10.0));}//100ms���ڵĵ�50ms
		if(count2==10){count2=0;}	
	}
	//10ms
	if(count3>=0 && count3<=200)//rgb����ɫ�仯
	{
		if(count3<=50)
		{
			PAout(5) = 1;
			PWM_LED_B = 140*count3;
			PWM_LED_G = 140*count3;//���ƺ��̵������𽥱��7000
		}
		if(count3>50&&count3<100) 
		{
			PAout(5)=0;//red
			PWM_LED_B = 7199-140*(count3-50);
			PWM_LED_G = 7199-140*(count3-50);//���ƺ��̵������𽥱�����0
		}
		if(count3>=100&&count3<150)//rgb����ɫ�仯
		{
			PAout(5)=1;
			PWM_LED_B = 140*(count3-100);
			PWM_LED_G = 140*(count3-100);//���ƺ��̵������𽥱��7000
		}
		if(count3>150&&count3<200) 
		{
			PAout(5)=0;//red
			PWM_LED_B = 7199-140*(count3-150);
			PWM_LED_G = 7199-140*(count3-150);//���ƺ��̵������𽥱�����0
		}
		if(count3==200){count3=0;}
	}
		
}

/*********************
ֱ����PD��������Kp*Ek+Kd*Ek_D

��ڣ������Ƕȡ���ʵ�Ƕȡ���ʵ���ٶ�
���ڣ�ֱ�������
*********************/
int Vertical_PD(float tar_Med,float rt_Angle,float gyro_x)
{
	float Bias;
	static float error;
	int PWM_out1;
	Bias = rt_Angle - tar_Med;
	error += Bias;
	
	if(error>30) {error = 30;}
	if(error<-30){error = -30;}
	
	PWM_out1 = Angle_Kp*Bias + Angle_Ki*error + Angle_Kd*(gyro_x);

	return PWM_out1;
}

/*********************
�ٶȻ�PI��Kp*Ek+Ki*Ek_S
*********************/
int Velocity_PI(int encoder_motor)
{
	static int Encoder_S,EnC_Err_Lowout;
	int EnC_Err_Lowout_last,PWM_out2;
	EnC_Err_Lowout_last=encoder_motor;//1.�����ٶ�ƫ��
    
	//2.���ٶ�ƫ����е�ͨ�˲�:low_out=(1-a)*Ek+a*low_out_last
	EnC_Err_Lowout*=0.7;						//ʹ�ò��θ���ƽ�����˳���Ƶ���ţ���ֹ�ٶ�ͻ�䡣
	EnC_Err_Lowout+=0.3*EnC_Err_Lowout_last;		//��ֹ�ٶȹ����Ӱ��ֱ����������������
	Encoder_S+=EnC_Err_Lowout;					//3.���ٶ�ƫ����֣����ֳ�λ��
	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);//4.�����޷�
	PWM_out2=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;//5.�ٶȻ������������
	return PWM_out2;
}

/*********************
�ٶȻ�PI:����ʽPID
*********************/
int Velocity_IncCtrl_PI (int Encoder1,int Target1)
{  
   static float Bias1,PWM1,Last_bias1;
   Bias1=Target1-Encoder1;                                  //����ƫ��
   PWM1+=10*(Bias1-Last_bias1)+0.5*Bias1;   			//����ʽPI������
   Last_bias1=Bias1;                                       //������һ��ƫ�� 
   return PWM1;                                           //�������
}



