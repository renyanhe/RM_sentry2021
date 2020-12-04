#ifndef _PWM_H
#define _PWM_H

#define SC_GPIO           GPIOC
#define SC_GPIO_Pin       GPIO_Pin_0
typedef enum status
{
   finish = 1,   //���
	 start,            //����������
	 catch_up,     //����������
	 catch_down,    //�����½���
	 long_time      // ����ʱ��
}status;

typedef struct ultrasonic
{
   status flag;
	 int distance;
}ult;


void PWM_init(void);
void PWM_iic_init(void);

#endif

