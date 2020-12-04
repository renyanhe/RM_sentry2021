#include "HC_ultrasonic.h"

	 TIM_ICInitTypeDef  TIM3_ICInitStructure;
	 TIM_ICInitTypeDef  TIM9_ICInitStructure;
	 GPIO_InitTypeDef GPIO_InitStructure;
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
//定时器3通道4,定时器9通道1输入捕获配置
void HC_Reci_TIM3_Init(u16 arr,u16 psc)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
	
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3时钟使能 
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTB时钟	
	 	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PB1定时器3通道4
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能

	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	 GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PB1

	 GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3); //PB1复用位定时器3
		   
	 TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	 TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	 TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	 TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	 TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	 
	//初始化TIM3通道4输入捕获参数
	 TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	选择输入端 IC4映射到TI4上
   TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
   TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
   TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
   TIM3_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
   TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	 
	 //允许更新中断，触发方式中断
	 TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	 TIM_ITConfig(TIM3,TIM_IT_Trigger,ENABLE);
 
   NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	 NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	 
	 TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC4,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
 
   TIM_Cmd(TIM3,ENABLE );  //使能定时器3
}

void HC_Reci_TIM9_Init(u16 arr,u16 psc)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	//TIM9时钟使能 
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTE时钟
   
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PE5定时器9通道1
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能

	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	 GPIO_Init(GPIOE,&GPIO_InitStructure); //初始化PE5

	 GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9); //PE5复用位定时器9
	
	 TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	 TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	 TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	 TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	 
	 TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);
	 //初始化TIM9通道1输入捕获参数
	 TIM9_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
   TIM9_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
   TIM9_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
   TIM9_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
   TIM9_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
   TIM_ICInit(TIM9, &TIM9_ICInitStructure);
	 
	 //允许更新中断，触发方式中断
	 TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE);
	 TIM_ITConfig(TIM9,TIM_IT_Trigger,ENABLE);
	 
	 NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	 NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	 
	 TIM_ITConfig(TIM9,TIM_IT_Update|TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	 
	 TIM_Cmd(TIM9,ENABLE );  //使能定时器9
}

void Send_sound(GPIO_TypeDef* GPIOx,uint16_t Pinx)
{
   GPIO_SetBits(GPIOx,Pinx);
	 delay_us(20);
	 GPIO_ResetBits(GPIOx,Pinx);
}

HC HC_1;//TIM3_4
HC HC_2;//TIM9_1
void TIM3_IRQHandler(void)//TIM3通道4
{
	 if((HC_1.sta&0X80)==0)//还未成功捕获	
	 {
	    if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//溢出
		  {	
         Send_sound(GPIOB,GPIO_Pin_0);				
			   if(HC_1.sta&0X40)//已经捕获到高电平了
			   {
				    if((HC_1.sta&0X3F)==0X3F)//高电平太长了
				    {
					     HC_1.sta|=0X80;		//标记成功捕获了一次
					     HC_1.val=0XFFFF;
				    }else HC_1.sta++;
			   }	 
		  }
      if(TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)//捕获1发生捕获事件
      {	
	       if(HC_1.sta&0X40)		//捕获到一个下降沿 		
			   {	  			
			      HC_1.sta|=0X80;		//标记成功捕获到一次高电平脉宽
			      HC_1.val=(u16)TIM_GetCapture4(TIM3);//获取当前的捕获值.
					 
					  HC_1.temp=HC_1.sta&0X3F; 
			      HC_1.temp*=0XFFFF;		 		         //溢出时间总和
			      HC_1.temp+=HC_1.val;			//得到总的高电平时间
			      HC_1.Distance=HC_1.temp*172/10000;//cm
			      HC_1.sta=0;			     //开启下一次捕获
					  printf("HC1: %d\n",HC_1.Distance);
	 			    TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
//					  Send_sound(GPIOB,GPIO_Pin_0);
			   }
			   else  								//还未开始,第一次捕获上升沿
			   {
				    HC_1.sta=0;			//清空
				    HC_1.val=0;
				    HC_1.sta|=0X40;		//标记捕获到了上升沿
				    TIM_Cmd(TIM3,DISABLE ); 	//关闭定时器3
	 			    TIM_SetCounter(TIM3,0);
	 			    TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				    TIM_Cmd(TIM3,ENABLE ); 	//使能定时器3
			   }		    
	    }			     	    					   
   }
   TIM_ClearITPendingBit(TIM3, TIM_IT_CC4|TIM_IT_Update);
}


void TIM1_BRK_TIM9_IRQHandler(void)//TIM9通道1
{	
   if((HC_2.sta&0X80)==0)//还未成功捕获	
	 {
	    if(TIM_GetITStatus(TIM9, TIM_IT_Update) != RESET)//溢出
		  {	 
				 Send_sound(GPIOE,GPIO_Pin_6);
			   if(HC_2.sta&0X40)//已经捕获到高电平了
			   {
				   if((HC_2.sta&0X3F)==0X3F)//高电平太长了
				   {
					    HC_2.sta|=0X80;		//标记成功捕获了一次
					    HC_2.val=0XFFFF;
				   }
				   else HC_2.sta++;
			   }	 
		  }
		  if(TIM_GetITStatus(TIM9, TIM_IT_CC1) != RESET)//捕获2发生捕获事件
		  {	
			   if(HC_2.sta&0X40)		//捕获到一个下降沿 		
			   {	  			
				    HC_2.sta|=0X80;		//标记成功捕获到一次高电平脉宽
			      HC_2.val=(u16)TIM_GetCapture1(TIM9);//获取当前的捕获值.
					  
					  HC_2.temp=HC_2.sta&0X3F; 
			      HC_2.temp*=0XFFFF;		 		         //溢出时间总和
			      HC_2.temp+=HC_2.val;			//得到总的高电平时间
			      HC_2.Distance=HC_2.temp*172/20000;//cm
			      HC_2.sta=0;			     //开启下一次捕获	
					  printf("HC2: %d\n",HC_2.Distance);
	 			    TIM_OC1PolarityConfig(TIM9,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
//					  Send_sound(GPIOE,GPIO_Pin_6);
			   }
				 else  								//还未开始,第一次捕获上升沿
			   {
				    HC_2.sta=0;			//清空
				    HC_2.val=0;
				    HC_2.sta|=0X40;		//标记捕获到了上升沿
				    TIM_Cmd(TIM9,DISABLE ); 	//关闭定时器5
	 			    TIM_SetCounter(TIM9,0);
	 			    TIM_OC1PolarityConfig(TIM9,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				    TIM_Cmd(TIM9,ENABLE ); 	//使能定时器5
			   }		    
		   }			     	    					   
 	  }
   TIM_ClearITPendingBit(TIM9, TIM_IT_CC1|TIM_IT_Update); //清除中断标志位
}

void HC_ult_send()
{
	 GPIO_InitTypeDef  GPIO_InitStructure;
	 GPIO_InitTypeDef  GPIO_InitStructure2;
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
	   
	//GPIOB0初始化设置，超声波发出，trig1
   GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
   GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	 GPIO_ResetBits(GPIOB,GPIO_Pin_0);//trig1对应引脚GPIOB0拉低
	
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTE时钟
	//GPIOE6初始化设置，超声波发出，trig2
	 GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_6;
   GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
   GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;//推挽输出
   GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
   GPIO_Init(GPIOE, &GPIO_InitStructure2);//初始化
	
	 GPIO_ResetBits(GPIOE,GPIO_Pin_6);//trig2对应引脚GPIOB0拉低
}
