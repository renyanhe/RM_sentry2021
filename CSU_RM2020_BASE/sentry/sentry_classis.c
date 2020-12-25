#include "sentry_classis.h"
/*
空载：PID_Struct_Init(&CAN2_DATA.M_201.PID_Speed ,10,2,10,10000);//内环速度环、外环电流环
			PID_Struct_Init(&CAN2_DATA.M_201.PID_I ,1,0.05,0,10000);
带载：PID_Struct_Init(&CAN2_DATA.M_202.PID_Speed ,14,2.1,15,10000);
      PID_Struct_Init(&CAN2_DATA.M_202.PID_I ,1.2,0.05,0.2,10000);
*/
void Init_classis()
{
   PID_Struct_Init(&CAN2_DATA.M_201.PID_Speed ,14,2.1,15,10000);//内环速度环、外环电流环
	 PID_Struct_Init(&CAN2_DATA.M_201.PID_I ,1.2,0.05,0.2,10000);
	 PID_Struct_Init(&CAN2_DATA.M_202.PID_Speed ,14,2.1,15,10000); 
	 PID_Struct_Init(&CAN2_DATA.M_202.PID_I ,1.2,0.05,0.2,10000);
}
u8 flag = RP_S_MID;
int16_t remote_speed;
float curspeed;
float cur_classis1,cur_classis2;
void Update_out()
{
	 p_sentry sentry;
	 sentry = msg_get_read_try(&sentry_msg);
	 if(sentry != NULL)
	 {
	    flag = sentry->SW_left;
		  remote_speed = sentry->speed;
	 }
	 else//遥控器保护
   {
	    CAN2_DATA.M_202.Tar_I = 0;
			CAN2_DATA.M_201.Tar_I = 0;	   
	 }
	 msg_read_finish(&sentry_msg);
	 
		if(flag == RP_S_DOWN)//底盘自动
	 {
//      if(HC_1.Distance<20 &&HC_2.Distance>20)
//	    {
//	       CAN2_DATA.M_202.Tar_speed = -1000;
//		     CAN2_DATA.M_201.Tar_speed = 1000;
//	    }
//	    else if(HC_1.Distance>20 &&HC_2.Distance<20)
//	    {
//	       CAN2_DATA.M_202.Tar_speed = 1000;
//		     CAN2_DATA.M_201.Tar_speed = -1000;	    
//	    }
		}
		if(flag == RP_S_MID)//左拨杆中为停止
	 {
			CAN2_DATA.M_202.Tar_I = 0;
			CAN2_DATA.M_201.Tar_I = 0;	    
	 }
		if(flag == RP_S_UP)
	 {
		  CAN2_DATA.M_202.Tar_speed = -4.5*remote_speed;
		  CAN2_DATA.M_202.Tar_I = PID_Update_Incre(&CAN2_DATA.M_202.PID_Speed,CAN2_DATA.M_202.Tar_speed,CAN2_DATA.M_202.Cur_speed);
		 
		  CAN2_DATA.M_201.Tar_speed =  4.5*remote_speed;	    
		  CAN2_DATA.M_201.Tar_I = PID_Update_Incre(&CAN2_DATA.M_201.PID_Speed,CAN2_DATA.M_201.Tar_speed,CAN2_DATA.M_201.Cur_speed);
	 } 
	 
	 CAN2_DATA.M_201.speed_out = PID_Update_Incre(&CAN2_DATA.M_201.PID_I,CAN2_DATA.M_201.Tar_I,CAN2_DATA.M_201.Cur_I);
   CAN2_DATA.M_202.speed_out = PID_Update_Incre(&CAN2_DATA.M_202.PID_I,CAN2_DATA.M_202.Tar_I,CAN2_DATA.M_202.Cur_I);
	 curspeed = CAN2_DATA.M_201.Cur_speed;
	 cur_classis1 = CAN2_DATA.M_201.Cur_I;
	 cur_classis2 = CAN2_DATA.M_202.Cur_I;
}
float aver_power = 0;
void Sentry_Classis(void *param)  
{
	 Init_classis();
	 while(1)
	 {
		  Update_out();
		  Can_out();
		  task_delay_ms(1);
			aver_power  = 0.8 * aver_power + 0.2 * judge_recv_mesg.ext_power_heat_data.chassis_power;
		  print_wave(2,4,&curspeed,&aver_power);
	 }
}