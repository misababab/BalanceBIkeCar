/* ���Ŀ��ƺ�����stm32f10x_it.c��ϵͳ�δ�ʱ�� */
#include "stm32f10x.h"
#include "sys.h"
#include "usart.h"
#include "usart2.h"
#include "motor.h"
#include "adc.h"
#include "PID.h"
#include "systick.h"
#include "mpu6050.h"
#include "mpuiic.h"
#include "delay.h"
#include "Servo.h"
#include "wave.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/*  ��ʱ����Ϊ���֣�PWM��ʱ������������ʱ���������ж϶�ʱ��
	�����֣�  ��������       PB12   ->   EN
			 ת����������    PB13   ->   DR
			 PWM�����ź�     PB0��TIM3-ch3��
			 ������AB��		 PB6	 ->   TIM4-ch1
							 PB7	 ->	  TIM4-ch2
	
	N20����� ��������		PC14
							PC15
			  PWM�����ź�	PB1��TIM3-ch4��
	
	RGB������ PWM�����ź�	PA6(G)��PA7(B)
				һ������			PA5(R)	
*/

/**
  * @brief  PWM��ʱ����
				TIM3-ch1��PA6 RGB���̵�
				TIM3-ch2��PA7 RGB������
				TIM3-ch3��PB0 ������PWM�����ź�
				TIM3-ch4��PB1 n20���PWM�����ź�
			��������ʱ����
				TIM4-ch1
				TIM4-ch2
			�����ж϶�ʱ����
				ϵͳ�δ�ʱ��10ms
  */

/**
  * @brief  ���ܿ�ܣ�����ʱ״̬����
				ֱ��ƽ�⡢
				�������п��ơ���ǰ�������ˡ���ת����ת�������ٶȡ�ת��Ƕȣ�
				S��·�α��ϡ��������������������������ȡ������·��Ȼ����ߣ�
				����ң�أ�stm32ң������������λ������
				ADC������ѹ
 */


int main()
{
	//uart1_Init(115200);
	usart2_Init(115200);
	motor_GtrlPin_init();				//���IO���ƶ˿ڳ�ʼ�� B12 B13 C14 C15
	TIM3_PWM_Motor_Led_Init(7199,0);  	//B0 B1
	TIM4_Encoder_wheel_Init(65535,0);	//B6 B7	
	adc_Battery_Init();             	//A4����ADC�����������ص�ѹ
	MPU_Init();							//mpu6050�����ǳ�ʼ��
	delay_ms(10);
	TIM1_PWM_Servo_Init(1999,719); 		//�����ʼ�� A8	A11
	delay_ms(10);
	while(mpu_dmp_init() )  		//����DMP��ʼ��
	{
		delay_ms(50);
	}	
	servo_CtrlCarTrun(86);				//����Ƕȹ���
	servo_CtrlWaveTrun(81);			//81
	delay_ms(10);
	
	TIM_SetCompare4(TIM3,2500);		//PB1 С����ʼ�ٶ�Ϊ2��
	motor_n20_Disable();			//��ʼ��Ϊֹͣ״̬
	
	Wave_CtrlPin_Init();			//���������ų�ʼ��
	TIM2_Update_Wave_Init();		//��������ʱ����ʼ��
	
	/* �δ�ʱ�������������� */
	SysTick_Init();			//SysTick������ʼ��	
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;//ʹ�����㷨
	while(1)
	{
		
	//AdcValue=11.09*(3.3*Get_adc_Average(ADC_Channel_4,10)/0x0fff); //ADCֵ��ΧΪ��0-2^12=4095��111111111111��һ������¶�Ӧ��ѹΪ0-3.3V
		/*��ʱ���stm32f10x_it.c�ĵδ�ʱ��������*/
		/*
		if(mpu_dmp_get_data(&Pitch,&Roll,&Yaw)==0){}//�Ƕȶ�ȡ//ע�⣬��û�в�������ʱ�ò���Ҫע�͵�
		temp=MPU_Get_Temperature();	//�õ��¶�ֵ//�¶�ֵ(������100��)
			printf("temp=%d\r\n",temp);
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
			printf("pitch=%.2f roll=%.2f yaw=%.2f\r\n",Pitch,Roll,Yaw);
			printf("aacx=%d aacy=%d aacz=%d\r\n",aacx,aacy,aacz);
			printf("gyrox=%d gyroy=%d gyroz=%d\r\n",gyrox,gyroy,gyroz);
		
		Encoder_DDL=-Read_Encoder(4);  //===��ȡ��������ֵ  
			printf("Encoder_A=%d Encoder_B=%d\r\n",Encoder_A,Encoder_B);
		PWMDDL=3599;//PA0 PWM���
		
	
		INA1=1;//PB12
		INA2=0;//PB13

		*/
//		PCout(13)=1;
//		delay_ms(500);
//		PCout(13)=0;
//		delay_ms(500);
		
		
		//printf2("1\r\n");
		//Distance=dist_sensor();
		Wave_Strat();
//		printf2("dis=%.1f\r\n",Distance);
//		printf2("U=%.2f \r\n",AdcValue);	
		//delay_ms(50);
		AdcValue=3.849*(3.3*Get_adc_Average(ADC_Channel_4,10)/0x0fff); 
	}
}

/**
  * @brief  ������PWM�źŵ͵�ƽ������RGB��Ҳ�ǵ͵�ƽ������PWM��ʱ����������Ч��ƽΪ�͵�ƽ

  */
