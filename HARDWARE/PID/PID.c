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

int Encoder_Wheel;          //有符号编码器的脉冲计数
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

float Med_Angle=0;//机械中值-能使得小车真正平衡住的角度。
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
int Vertical_PWM,Velocity_PWM;//直立环&速度环的输出变量

void control(void)//10ms执行一次
{
	int PWM_out;
	/* 获取速度 */
	Encoder_Wheel = -Read_Encoder(4);  //从编码器中读取当前轮子的速度（或位置），并取反以得到实际的速度值。  
	/* 获取角度、角速度  */
	mpu_dmp_get_data(&pitch,&roll,&yaw);			//角度
	
	if(roll>=0) {PCout(13)=1;}
	if(roll<0)  {PCout(13)=0;}
	
	/* PID平衡控制 */
	Vertical_PWM = Vertical_PD(Med_Angle,roll,gyro[0]); /* 直立环 */
	Velocity_PWM = Velocity_PI(Encoder_Wheel);		    /* 速度环 */	
	PWM_out = Vertical_PWM + Velocity_PWM;				/* 最终输出 */
	
	if(roll<-15||roll>15)//判断车身倒下
	{
		PWM_out = Velocity_IncCtrl_PI(Encoder_Wheel,0);	//动量轮速度强制设为零
		count3 = -2;//关闭rgb灯颜色变化
		openRedLed();//亮红灯
	}		
	Limit(&PWM_out);	
	Load(PWM_out);//加载到电机上。
	
	/* 10ms +1 */
	count1++;
	count2++; 
	count3++;
	count4++;

	//每50ms读取一次串口值
	if(count1==5)
	{
		count1=0;
		
		if(usart2_CacheByte=='y'|| usart2_CacheByte=='b')//判断遥控器类型（100：电脑上位机、stm32遥控器）
		{
			usart2_flag=100;
		}
		if(usart2_CacheByte=='b'|| usart2_flag==7)//避障模式
		{ 
			bizhang_mode=1; 
		}
		if(usart2_CacheByte=='y'|| usart2_flag==6)//遥控模式
		{ 
			bizhang_mode=0;  
			usart2_CacheByte='0'; //进入遥控模式后，停车等待用户
		}
		
		if(bizhang_mode==0)//遥控模式
		{
			/********避障模式的变量清零*******/
			turn_stop_flag=0;
			delay1=0;
			delay2=0;
			delay3=0;
			delay4=0;
			/*********************************/
			/* 轮子前进OR后退 */
			if(usart2_CacheByte=='0'|usart2_flag==0){motor_n20_Disable();}//停
			if(usart2_CacheByte=='1'|usart2_flag==1){motor_n20_Forward();}//前进
			if(usart2_CacheByte=='2'|usart2_flag==2){motor_n20_Back();}//后退
			/* 速度 */
			if(usart2_CacheByte=='A'){TIM_SetCompare4(TIM3,7199);}				 //小车速度0档（停）
			if(usart2_CacheByte=='B'|usart2_flag==8){TIM_SetCompare4(TIM3,4000);}//小车速度1档
			if(usart2_CacheByte=='C'|usart2_flag==9){TIM_SetCompare4(TIM3,2500);}//小车速度2档
			if(usart2_CacheByte=='D'|usart2_flag==10){TIM_SetCompare4(TIM3,1000);}//小车速度3档
			if(usart2_CacheByte=='E'|usart2_flag==11){TIM_SetCompare4(TIM3,1);}//小车速度4档（最快）
			/* 车头方向 */
			if(usart2_CacheByte=='F'){servo_CtrlCarTrun(86-50);}//左转45度
			if(usart2_CacheByte=='G'){servo_CtrlCarTrun(86-44);}//左转40度
			if(usart2_CacheByte=='H'){servo_CtrlCarTrun(86-38);}//左转35度
			if(usart2_CacheByte=='I'){servo_CtrlCarTrun(86-32);}//左转30度
			if(usart2_CacheByte=='J'|usart2_flag==4){servo_CtrlCarTrun(86-27);}//左转25度
			if(usart2_CacheByte=='K'){servo_CtrlCarTrun(86-22);}//左转20度
			if(usart2_CacheByte=='L'){servo_CtrlCarTrun(86-16);}//左转15度
			if(usart2_CacheByte=='M'){servo_CtrlCarTrun(86-11);}//左转10度
			if(usart2_CacheByte=='N'){servo_CtrlCarTrun(86-6);}//左转5度
			
			if(usart2_CacheByte=='O'|usart2_flag==3){servo_CtrlCarTrun(86);}  //车头摆正
			
			if(usart2_CacheByte=='P'){servo_CtrlCarTrun(86+6);} //右转5度
			if(usart2_CacheByte=='Q'){servo_CtrlCarTrun(86+11);}//右转10度
			if(usart2_CacheByte=='R'){servo_CtrlCarTrun(86+16);}//右转15度
			if(usart2_CacheByte=='S'){servo_CtrlCarTrun(86+22);}//右转20度
			if(usart2_CacheByte=='T'|usart2_flag==5){servo_CtrlCarTrun(86+27);}//右转25度
			if(usart2_CacheByte=='U'){servo_CtrlCarTrun(86+32);}//右转30度
			if(usart2_CacheByte=='V'){servo_CtrlCarTrun(86+38);}//右转35度
			if(usart2_CacheByte=='W'){servo_CtrlCarTrun(86+44);}//右转40度
			if(usart2_CacheByte=='X'){servo_CtrlCarTrun(86+50);}//右转45度
		}
		if(bizhang_mode==1)//避障模式
		{		
			TIM_SetCompare4(TIM3,2500);//速度二档进行避障
			/* 前方无障碍 */
			if(Distance>25&&turn_stop_flag==0)
			{
				PCout(14)=0;PCout(15)=1;	//前进
				count3=-2;					//关闭rgb灯颜色变化
				openGreenLed();				//亮绿灯
			}
			/* 遇到障碍物 */
			if(Distance<=25&turn_stop_flag==0)
			{
				PCout(14)=0;PCout(15)=0;	//停
				count3=-2;					//关闭rgb灯颜色变化
				openBlueLed();				//亮蓝灯
				
				turn_stop_flag=1;
			}
			if(turn_stop_flag==1)
			{
				delay1++;//50ms自增
				/* 状态1 左摆头计算距离 */
				/* delay1状态(delay1<10)++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
				if(delay1<10)			   			//delay1计时、延时0-500ms测量左边障碍物时间	  
				{
					servo_CtrlWaveTrun(165);	 	/* Wave左摆头 */
				}
				if(delay1==10)
				{
					left_distance = Distance;		/* 获取Wave左摆头距离 */
					servo_CtrlWaveTrun(0);		 	/* Wave右摆头 */
				}
				/* 状态2 右摆头计算距离 */
				/* delay2状态 (delay2<=10 && delay1>10)++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
				if(delay1>=10){delay2++;delay1=11;} //delay2计时、延时0-500ms测量左边障碍物时间
				if(delay2==10)	
				{
					right_distance = Distance; 	 	/* 获取Wave右摆头距离*/
					servo_CtrlWaveTrun(81);		 	/* 摆正Wave */
				}
				/* 状态3 转弯策略 */			
				/* delay3状态 (delay3<=5 && delay2>10)+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
				if(delay2>=10){delay3++;delay2=11;}	//delay3计时、延时0-250ms		
				if(delay3==5)//进入判断车头摆左还是右
				{
					if(left_distance>=right_distance && left_distance>30)/* 左边无障碍，车头摆左 */
					{
						servo_CtrlCarTrun(86-39);//左转40度
					}
					
					if(left_distance<right_distance && right_distance>30)/* 右边无障碍，车头摆右 */
					{
						servo_CtrlCarTrun(86+40);//右转40度 
					}
					
					if(left_distance<=30 && right_distance<=30)/* 左右都有障碍物，车头摆直，到达目的地停车 */
					{
						motor_n20_Disable();//停
						servo_CtrlCarTrun(86);//摆正车头
						count3 = -2;	//关闭rgb灯颜色变化
						openRedLed();//亮红灯
			
						delay3 = 0;/* 将Delay3清零不进行后续操作，状态卡在3 */
					}
				}
				/* 状态4 驱动车辆转弯一段距离 */
				/* delay4状态 (delay4<=35 && delay3>5)+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
				if(delay3>=5){delay4++;delay3=6;}//delay4计时、延时1750ms	
				if(delay3>=5 && delay4<35)
				{
					PCout(14)=0;PCout(15)=1;//前进
				}
				if(delay4==35)
				{
					motor_n20_Disable();		//停
					servo_CtrlCarTrun(86);		//车头摆正
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
	if(usart2_flag!=100)//stm32遥控器遥控。小车上传数据给遥控器
	{
		if(count2==100)//每1000毫秒串口发送电池电压AD采样
		{			
			count2=0;
			
			printf2("t16.txt=\"%.1f\"",AdcValue); 
			printf2("\xff\xff\xff");	//包尾，隔离两个有效数据	
		}
		if(count4==50)//500ms发送一次角度和距离信息
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
	if(usart2_flag==100)//电脑上位机遥控小车
	{	
		if(count2%2==0){printf2("r%d",(int)(roll*100.0));}//每20毫秒串口发送一次测距距离			
		if(count2==3){printf2("v%d",(int)(AdcValue*100.0));}//100ms周期的第30ms
		if(count2==5){printf2("d%d",(int)(Distance*10.0));}//100ms周期的第50ms
		if(count2==10){count2=0;}	
	}
	//10ms
	if(count3>=0 && count3<=200)//rgb灯颜色变化
	{
		if(count3<=50)
		{
			PAout(5) = 1;
			PWM_LED_B = 140*count3;
			PWM_LED_G = 140*count3;//蓝灯和绿灯亮度逐渐变大到7000
		}
		if(count3>50&&count3<100) 
		{
			PAout(5)=0;//red
			PWM_LED_B = 7199-140*(count3-50);
			PWM_LED_G = 7199-140*(count3-50);//蓝灯和绿灯亮度逐渐变弱到0
		}
		if(count3>=100&&count3<150)//rgb灯颜色变化
		{
			PAout(5)=1;
			PWM_LED_B = 140*(count3-100);
			PWM_LED_G = 140*(count3-100);//蓝灯和绿灯亮度逐渐变大到7000
		}
		if(count3>150&&count3<200) 
		{
			PAout(5)=0;//red
			PWM_LED_B = 7199-140*(count3-150);
			PWM_LED_G = 7199-140*(count3-150);//蓝灯和绿灯亮度逐渐变弱到0
		}
		if(count3==200){count3=0;}
	}
		
}

/*********************
直立环PD控制器：Kp*Ek+Kd*Ek_D

入口：期望角度、真实角度、真实角速度
出口：直立环输出
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
速度环PI：Kp*Ek+Ki*Ek_S
*********************/
int Velocity_PI(int encoder_motor)
{
	static int Encoder_S,EnC_Err_Lowout;
	int EnC_Err_Lowout_last,PWM_out2;
	EnC_Err_Lowout_last=encoder_motor;//1.计算速度偏差
    
	//2.对速度偏差进行低通滤波:low_out=(1-a)*Ek+a*low_out_last
	EnC_Err_Lowout*=0.7;						//使得波形更加平滑，滤除高频干扰，防止速度突变。
	EnC_Err_Lowout+=0.3*EnC_Err_Lowout_last;		//防止速度过大的影响直立环的正常工作。
	Encoder_S+=EnC_Err_Lowout;					//3.对速度偏差积分，积分出位移
	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);//4.积分限幅
	PWM_out2=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;//5.速度环控制输出计算
	return PWM_out2;
}

/*********************
速度环PI:增量式PID
*********************/
int Velocity_IncCtrl_PI (int Encoder1,int Target1)
{  
   static float Bias1,PWM1,Last_bias1;
   Bias1=Target1-Encoder1;                                  //计算偏差
   PWM1+=10*(Bias1-Last_bias1)+0.5*Bias1;   			//增量式PI控制器
   Last_bias1=Bias1;                                       //保存上一次偏差 
   return PWM1;                                           //增量输出
}



