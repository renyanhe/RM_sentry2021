#include "sentry_Friction.h"

u16 pwmval=0;
//U8 mode;//����
U8 flag_pull;

void sentry_friction()
{
//	 PWM_init();
//	 PWM_iic_init();		//��base���ʼ��
		 pwmval= 1000;	//����1100
	 while(1)
	 {
//ʹ��UI����Ħ����ռ�ձȣ���ռ��ң����
			TIM1->CCR1 = pwmval;
			TIM1->CCR2 = pwmval;
			TIM1->CCR3 = pwmval;
			TIM1->CCR4 = pwmval;
		 
			TIM4->CCR1 = pwmval;
			TIM4->CCR2 = pwmval;
			TIM4->CCR3 = pwmval;
			TIM4->CCR4 = pwmval;
			task_delay_ms(2000);
	 }
}
