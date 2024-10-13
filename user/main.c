/* 核心控制函数在stm32f10x_it.c的系统滴答定时器 */
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

/*  定时器分为三种：PWM定时器、编码器定时器、更新中断定时器
	动量轮：  控制引脚       PB12   ->   EN
			 转动方向引脚    PB13   ->   DR
			 PWM驱动信号     PB0（TIM3-ch3）
			 编码器AB相		 PB6	 ->   TIM4-ch1
							 PB7	 ->	  TIM4-ch2
	
	N20电机： 控制引脚		PC14
							PC15
			  PWM驱动信号	PB1（TIM3-ch4）
	
	RGB呼吸灯 PWM驱动信号	PA6(G)、PA7(B)
				一般驱动			PA5(R)	
*/

/**
  * @brief  PWM定时器：
				TIM3-ch1：PA6 RGB的绿灯
				TIM3-ch2：PA7 RGB的蓝灯
				TIM3-ch3：PB0 动量轮PWM驱动信号
				TIM3-ch4：PB1 n20电机PWM驱动信号
			编码器定时器：
				TIM4-ch1
				TIM4-ch2
			更新中断定时器：
				系统滴答定时器10ms
  */

/**
  * @brief  功能框架：（计时状态机）
				直立平衡、
				车辆运行控制、（前进、后退、左转、右转、调节速度、转弯角度）
				S型路段避障、（舵机带动超声波传感器器获取左、右两路，然后决策）
				蓝牙遥控（stm32遥控器、电脑上位机）、
				ADC测量电压
 */


int main()
{
	//uart1_Init(115200);
	usart2_Init(115200);
	motor_GtrlPin_init();				//电机IO控制端口初始化 B12 B13 C14 C15
	TIM3_PWM_Motor_Led_Init(7199,0);  	//B0 B1
	TIM4_Encoder_wheel_Init(65535,0);	//B6 B7	
	adc_Battery_Init();             	//A4引脚ADC采样，计算电池电压
	MPU_Init();							//mpu6050陀螺仪初始化
	delay_ms(10);
	TIM1_PWM_Servo_Init(1999,719); 		//舵机初始化 A8	A11
	delay_ms(10);
	while(mpu_dmp_init() )  		//进行DMP初始化
	{
		delay_ms(50);
	}	
	servo_CtrlCarTrun(86);				//舵机角度归中
	servo_CtrlWaveTrun(81);			//81
	delay_ms(10);
	
	TIM_SetCompare4(TIM3,2500);		//PB1 小车初始速度为2档
	motor_n20_Disable();			//开始车为停止状态
	
	Wave_CtrlPin_Init();			//超声波引脚初始化
	TIM2_Update_Wave_Init();		//超声波定时器初始化
	
	/* 滴答定时器定义放在最后面 */
	SysTick_Init();			//SysTick函数初始化	
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;//使能总算法
	while(1)
	{
		
	//AdcValue=11.09*(3.3*Get_adc_Average(ADC_Channel_4,10)/0x0fff); //ADC值范围为从0-2^12=4095（111111111111）一般情况下对应电压为0-3.3V
		/*总时序见stm32f10x_it.c的滴答定时器处理函数*/
		/*
		if(mpu_dmp_get_data(&Pitch,&Roll,&Yaw)==0){}//角度读取//注意，当没有插陀螺仪时该部分要注释掉
		temp=MPU_Get_Temperature();	//得到温度值//温度值(扩大了100倍)
			printf("temp=%d\r\n",temp);
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			printf("pitch=%.2f roll=%.2f yaw=%.2f\r\n",Pitch,Roll,Yaw);
			printf("aacx=%d aacy=%d aacz=%d\r\n",aacx,aacy,aacz);
			printf("gyrox=%d gyroy=%d gyroz=%d\r\n",gyrox,gyroy,gyroz);
		
		Encoder_DDL=-Read_Encoder(4);  //===读取编码器的值  
			printf("Encoder_A=%d Encoder_B=%d\r\n",Encoder_A,Encoder_B);
		PWMDDL=3599;//PA0 PWM输出
		
	
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
  * @brief  动量轮PWM信号低电平驱动、RGB灯也是低电平驱动、PWM定时器设置了有效电平为低电平

  */
