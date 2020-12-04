#ifndef _PWM_H
#define _PWM_H

#define SC_GPIO           GPIOC
#define SC_GPIO_Pin       GPIO_Pin_0
typedef enum status
{
   finish = 1,   //完成
	 start,            //发出超声波
	 catch_up,     //捕获上升沿
	 catch_down,    //捕获下降沿
	 long_time      // 超过时间
}status;

typedef struct ultrasonic
{
   status flag;
	 int distance;
}ult;


void PWM_init(void);
void PWM_iic_init(void);

#endif

