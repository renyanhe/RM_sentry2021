#include "sentry_holder.h"

void vision_init()
{
   PID_Struct_Init(&CAN1_DATA.M_205.PID_Speed ,160,8,40,10000);
	 PID_Struct_Init(&CAN1_DATA.M_205.PID_Postion ,1,0,0,10000);
	 PID_Struct_Init(&CAN1_DATA.M_208.PID_Speed ,100,4,0,10000);
	 PID_Struct_Init(&CAN1_DATA.M_208.PID_Postion ,1,0,0,10000);
}
void vupdata()
{
	 CAN1_DATA.M_205.Tar_speed = PID_vision(&CAN1_DATA.M_205.PID_Postion,-yaw_new2020);
	 CAN1_DATA.M_205.speed_out = PID_Update_Incre(&CAN1_DATA.M_205.PID_Speed,CAN1_DATA.M_205.Tar_speed,CAN1_DATA.M_205.Cur_speed);
	 
	 CAN1_DATA.M_208.Tar_speed = PID_vision(&CAN1_DATA.M_208.PID_Postion,-pitch_new2020);
	 CAN1_DATA.M_208.speed_out = PID_Update_Incre(&CAN1_DATA.M_208.PID_Speed,CAN1_DATA.M_208.Tar_speed,CAN1_DATA.M_208.Cur_speed);
}

void sentry_vision(void *param)
{
   vision_init();
	 while(1)
	 {
	   vupdata();
		 Can_out();
		 task_delay_ms(1);     		 
	 }
}