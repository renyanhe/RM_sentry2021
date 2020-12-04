#include "sentry_ultrasonic.h"

u16 AmplitudeLimiterFilter1(u16 New_value,u16 *Last_value)//限幅滤波
{
	 u16 return_value;
	 if((New_value-(*Last_value))>=ERR||(New_value-(*Last_value))<=-ERR)  
	 {
	    return_value = *Last_value;
	 }
	 else
	 {
	    return_value = New_value;
	 }
	 *Last_value = New_value;
	 return return_value;
}

u16 position(u16 x1, u16 x2)//加权定位
{
	 u16 k,x,s=x1+x2+70,l=70;//假设s，L
	 if(x1<x2)
	 {
		  k=(x2-x1)/(2*s)+0.5;
		  x = k*(x1+l/2)+(1-k)*(s-l/2-x2);
	 }
	 if(x1>x2)
	 {
		  k = (x1-x2)/(2*s)+0.5;
		  x = (1-k)*(x1+l/2)+k*(s-l/2-x2);
	 }
	 if(x1==x2)
	 {
		  x = x1+l/2;
	 }
	 if((x1+x2)>160) 
   {x=200-x2-30;}
	 return x;
}

void sentry_ultrasonic ()
{

	 HC_ult_send();
 	 HC_Reci_TIM3_Init(0XFFFF,84-1); //以1Mhz的频率计数 1us计数
	 HC_Reci_TIM9_Init(0XFFFF,84-1);
   	while(1)
   {		 
      task_delay_ms(50);
	 }	
}
