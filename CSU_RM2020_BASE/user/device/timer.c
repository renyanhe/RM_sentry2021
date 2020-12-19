#include "timer.h"

void Count_init()
{
	 NVIC_InitTypeDef NVIC_InitStructure;
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
   TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	
	 TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
   TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInitStruct.TIM_Period = 2000;
   TIM_TimeBaseInitStruct.TIM_Prescaler = (84-1);
   TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);
	 TIM_SetCounter(TIM5,0);
	
	 TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); 
	 TIM_Cmd(TIM5,ENABLE); 
	
	 NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	 NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	 NVIC_Init(&NVIC_InitStructure);
}
volatile u8 frame = 0;
volatile float speed = 0;
volatile int last_pos = 0;
void TIM5_IRQHandler(void)
{
	 //static int count = 0;
   if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)//���
	 {
		 speed = (float)(CAN1_DATA.M_206.angle - last_pos)*14.6484375f/4;
		 last_pos = CAN1_DATA.M_206.angle;
		/*  if(count++ == 1000)
			{
			   count = 0;
				 frame = __count;
				 __count = 0;
			}*/
	    TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
	 }
}
//tim11΢����ʱ
void Delay_Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
    
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInitStruct.TIM_Period = 100-1;
    TIM_TimeBaseInitStruct.TIM_Prescaler = (84-1);
    TIM_TimeBaseInit(TIM11, &TIM_TimeBaseInitStruct);
    
    while((TIM11->SR & TIM_FLAG_Update)!=SET);
    TIM11->SR = (uint16_t)~TIM_FLAG_Update;
}
//0��100us��ʱ
void delay_us(uint32_t us_cnt)
{
	LIMIT(us_cnt,100,1);
    TIM11->CNT = us_cnt-1;
    TIM11->CR1 |= TIM_CR1_CEN;    
    while((TIM11->SR & TIM_FLAG_Update)!=SET);
    TIM11->SR = (uint16_t)~TIM_FLAG_Update;
    TIM11->CR1 &= ~TIM_CR1_CEN;
}
//���can�жϵ�ʱ��
void TIM12_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  ///ʹ��TIM12ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE); //����ʱ��12�����ж�
	TIM_Cmd(TIM12,ENABLE); //ʹ�ܶ�ʱ��12
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM8_BRK_TIM12_IRQn; //��ʱ��12�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//��ʱ��12�жϷ�����
can_rate_t canrate;
void TIM8_BRK_TIM12_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM12,TIM_IT_Update)==SET)
    {
		canrate.send=canrate.inc;
		memset(&canrate.inc,0,sizeof(canrate.inc));
    }                
    TIM_ClearITPendingBit(TIM12,TIM_IT_Update);
}
