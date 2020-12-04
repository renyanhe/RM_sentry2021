#include "HC_ultrasonic.h"

	 TIM_ICInitTypeDef  TIM3_ICInitStructure;
	 TIM_ICInitTypeDef  TIM9_ICInitStructure;
	 GPIO_InitTypeDef GPIO_InitStructure;
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
//��ʱ��3ͨ��4,��ʱ��9ͨ��1���벶������
void HC_Reci_TIM3_Init(u16 arr,u16 psc)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
	
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3ʱ��ʹ�� 
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��PORTBʱ��	
	 	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PB1��ʱ��3ͨ��4
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���

	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	 GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PB1

	 GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3); //PB1����λ��ʱ��3
		   
	 TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	 TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	 TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	 TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	 TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	 
	//��ʼ��TIM3ͨ��4���벶�����
	 TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	ѡ������� IC4ӳ�䵽TI4��
   TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
   TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
   TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
   TIM3_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
   TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	 
	 //��������жϣ�������ʽ�ж�
	 TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	 TIM_ITConfig(TIM3,TIM_IT_Trigger,ENABLE);
 
   NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	 NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	 
	 TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC4,ENABLE);//��������ж� ,����CC1IE�����ж�	
 
   TIM_Cmd(TIM3,ENABLE );  //ʹ�ܶ�ʱ��3
}

void HC_Reci_TIM9_Init(u16 arr,u16 psc)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	//TIM9ʱ��ʹ�� 
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTEʱ��
   
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PE5��ʱ��9ͨ��1
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���

	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	 GPIO_Init(GPIOE,&GPIO_InitStructure); //��ʼ��PE5

	 GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9); //PE5����λ��ʱ��9
	
	 TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	 TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	 TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	 TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	 
	 TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);
	 //��ʼ��TIM9ͨ��1���벶�����
	 TIM9_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
   TIM9_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
   TIM9_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
   TIM9_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
   TIM9_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
   TIM_ICInit(TIM9, &TIM9_ICInitStructure);
	 
	 //��������жϣ�������ʽ�ж�
	 TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE);
	 TIM_ITConfig(TIM9,TIM_IT_Trigger,ENABLE);
	 
	 NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	 NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	 
	 TIM_ITConfig(TIM9,TIM_IT_Update|TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�	
	 
	 TIM_Cmd(TIM9,ENABLE );  //ʹ�ܶ�ʱ��9
}

void Send_sound(GPIO_TypeDef* GPIOx,uint16_t Pinx)
{
   GPIO_SetBits(GPIOx,Pinx);
	 delay_us(20);
	 GPIO_ResetBits(GPIOx,Pinx);
}

HC HC_1;//TIM3_4
HC HC_2;//TIM9_1
void TIM3_IRQHandler(void)//TIM3ͨ��4
{
	 if((HC_1.sta&0X80)==0)//��δ�ɹ�����	
	 {
	    if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//���
		  {	
         Send_sound(GPIOB,GPIO_Pin_0);				
			   if(HC_1.sta&0X40)//�Ѿ����񵽸ߵ�ƽ��
			   {
				    if((HC_1.sta&0X3F)==0X3F)//�ߵ�ƽ̫����
				    {
					     HC_1.sta|=0X80;		//��ǳɹ�������һ��
					     HC_1.val=0XFFFF;
				    }else HC_1.sta++;
			   }	 
		  }
      if(TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)//����1���������¼�
      {	
	       if(HC_1.sta&0X40)		//����һ���½��� 		
			   {	  			
			      HC_1.sta|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			      HC_1.val=(u16)TIM_GetCapture4(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
					 
					  HC_1.temp=HC_1.sta&0X3F; 
			      HC_1.temp*=0XFFFF;		 		         //���ʱ���ܺ�
			      HC_1.temp+=HC_1.val;			//�õ��ܵĸߵ�ƽʱ��
			      HC_1.Distance=HC_1.temp*172/10000;//cm
			      HC_1.sta=0;			     //������һ�β���
					  printf("HC1: %d\n",HC_1.Distance);
	 			    TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
//					  Send_sound(GPIOB,GPIO_Pin_0);
			   }
			   else  								//��δ��ʼ,��һ�β���������
			   {
				    HC_1.sta=0;			//���
				    HC_1.val=0;
				    HC_1.sta|=0X40;		//��ǲ�����������
				    TIM_Cmd(TIM3,DISABLE ); 	//�رն�ʱ��3
	 			    TIM_SetCounter(TIM3,0);
	 			    TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
				    TIM_Cmd(TIM3,ENABLE ); 	//ʹ�ܶ�ʱ��3
			   }		    
	    }			     	    					   
   }
   TIM_ClearITPendingBit(TIM3, TIM_IT_CC4|TIM_IT_Update);
}


void TIM1_BRK_TIM9_IRQHandler(void)//TIM9ͨ��1
{	
   if((HC_2.sta&0X80)==0)//��δ�ɹ�����	
	 {
	    if(TIM_GetITStatus(TIM9, TIM_IT_Update) != RESET)//���
		  {	 
				 Send_sound(GPIOE,GPIO_Pin_6);
			   if(HC_2.sta&0X40)//�Ѿ����񵽸ߵ�ƽ��
			   {
				   if((HC_2.sta&0X3F)==0X3F)//�ߵ�ƽ̫����
				   {
					    HC_2.sta|=0X80;		//��ǳɹ�������һ��
					    HC_2.val=0XFFFF;
				   }
				   else HC_2.sta++;
			   }	 
		  }
		  if(TIM_GetITStatus(TIM9, TIM_IT_CC1) != RESET)//����2���������¼�
		  {	
			   if(HC_2.sta&0X40)		//����һ���½��� 		
			   {	  			
				    HC_2.sta|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			      HC_2.val=(u16)TIM_GetCapture1(TIM9);//��ȡ��ǰ�Ĳ���ֵ.
					  
					  HC_2.temp=HC_2.sta&0X3F; 
			      HC_2.temp*=0XFFFF;		 		         //���ʱ���ܺ�
			      HC_2.temp+=HC_2.val;			//�õ��ܵĸߵ�ƽʱ��
			      HC_2.Distance=HC_2.temp*172/20000;//cm
			      HC_2.sta=0;			     //������һ�β���	
					  printf("HC2: %d\n",HC_2.Distance);
	 			    TIM_OC1PolarityConfig(TIM9,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
//					  Send_sound(GPIOE,GPIO_Pin_6);
			   }
				 else  								//��δ��ʼ,��һ�β���������
			   {
				    HC_2.sta=0;			//���
				    HC_2.val=0;
				    HC_2.sta|=0X40;		//��ǲ�����������
				    TIM_Cmd(TIM9,DISABLE ); 	//�رն�ʱ��5
	 			    TIM_SetCounter(TIM9,0);
	 			    TIM_OC1PolarityConfig(TIM9,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
				    TIM_Cmd(TIM9,ENABLE ); 	//ʹ�ܶ�ʱ��5
			   }		    
		   }			     	    					   
 	  }
   TIM_ClearITPendingBit(TIM9, TIM_IT_CC1|TIM_IT_Update); //����жϱ�־λ
}

void HC_ult_send()
{
	 GPIO_InitTypeDef  GPIO_InitStructure;
	 GPIO_InitTypeDef  GPIO_InitStructure2;
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��
	   
	//GPIOB0��ʼ�����ã�������������trig1
   GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
   GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
	 GPIO_ResetBits(GPIOB,GPIO_Pin_0);//trig1��Ӧ����GPIOB0����
	
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTEʱ��
	//GPIOE6��ʼ�����ã�������������trig2
	 GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_6;
   GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
   GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;//�������
   GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
   GPIO_Init(GPIOE, &GPIO_InitStructure2);//��ʼ��
	
	 GPIO_ResetBits(GPIOE,GPIO_Pin_6);//trig2��Ӧ����GPIOB0����
}
