#include "testbeach.h"
#define CIRCLE_2006 ((float)circle_2006.circle + (float)circle_2006.angle/8192)
u16 pitch = 0;
u16 yaw = 0;
PID_IncreType pos,speed;
float pos_tar = 0;
float speed_tar = 0;
s16 out = 0;
extern volatile int16_t speed_2006;
u8 Can_out[8];
void test_beach(void *param)
{
 //  p_sentry sentry;
	 while(1)
	 {
		  pos_tar = 18;
	   /* sentry = msg_get_read_try(&sentry_msg);
		  if(sentry != NULL)
			{
			   pitch = sentry->pitch;
				 yaw = sentry->yaw;
				 msg_read_finish_try(&sentry_msg);
			}*/
			task_delay_ms(2000);
	 }
}

void test_beach1(void *param)
{
   PID_Struct_Init(&pos,40,0,0,10000);
	 PID_Struct_Init(&speed,10,1,0,10000);
	 task_delay_ms(1000);
	 CanTxMsg	canmsg;
	 while(1)
	 {
	   // pos_tar += 3;
		  speed_tar = PID_Update_Incre(&pos,pos_tar,CIRCLE_2006);
		  out = (s16)PID_Update_Incre(&speed,speed_tar,speed_2006);
		  
	    Can_out[0] = (char)(out>>8);
	    Can_out[1] = (char)(out);
	    can_set(&canmsg, 0x200,Can_out);	
	    CAN_Transmit(CAN1,&canmsg);	
		  task_delay_ms(1);
	 }
}
#define W (float)(2 * PI * freq)  //角频率
	
float freq = 0.5;  //频率hz
float A = 1000; //幅值
float OUT = 0; //输出量
void test_beach2(void *param)
{
	static float count = 0;
	float t;
   while(1)
	{
		 t = count++/1000;
	   OUT = A * sin(W * t);
		 if(W*t >= 2*PI)
		 {
		    count = 0;
		 }
		 task_delay_ms(1);
	}
}