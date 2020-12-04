#include "sentry_Friction.h"

u16 pwmval=0;
//U8 mode;//拨杆
U8 flag_pull;

void sentry_friction()
{
//	 PWM_init();
//	 PWM_iic_init();		//在base里初始化
		 pwmval= 1000;	//大于1100
	 while(1)
	 {
//使用UI控制摩擦轮占空比，不占用遥控器
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
