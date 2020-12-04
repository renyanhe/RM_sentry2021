#include "sentry_pull.h"


void Init_pull()
{
   PID_Struct_Init(&CAN2_DATA.M_203.PID_Postion,50,0,1,10000);
	 PID_Struct_Init(&CAN2_DATA.M_203.PID_Speed ,5,0.02,0,10000);
   PID_Struct_Init(&CAN2_DATA.M_204.PID_Postion,50,0,1,10000);
	 PID_Struct_Init(&CAN2_DATA.M_204.PID_Speed ,5,0.02,0,10000);
	 CAN2_DATA.M_203.Tar_Postion = 0;
	 CAN2_DATA.M_204.Tar_Postion = 0;
}

void Updata_pos()
{
   p_sentry sentry;
	 sentry = msg_get_read_try(&sentry_msg);
	 if(sentry != NULL)
	 {
	    if(sentry->SL_pull == SHOOT)
			{
			   CAN2_DATA.M_203.Tar_Postion += 4; //逆时针转动
			}
			else if(sentry->SL_pull == SHOOT_2)
			{
			   CAN2_DATA.M_204.Tar_Postion += 4;
			}
	 }
	 else
   {
	    CAN2_DATA.M_203.Tar_Postion = CAN2_DATA.M_203.Cur_postion;
	    CAN2_DATA.M_204.Tar_Postion = CAN2_DATA.M_204.Cur_postion;
	 }
	 msg_read_finish_try(&sentry_msg);
}

void Updata_out()
{
	 if(CAN2_DATA.M_203.Tar_Postion - CAN2_DATA.M_203.Cur_postion > 0.5)
	 {
	    CAN2_DATA.M_203.Tar_speed = 3400;
	 }
	 else if(CAN2_DATA.M_203.Tar_Postion - CAN2_DATA.M_203.Cur_postion < -0.5)
	 {
	    CAN2_DATA.M_203.Tar_speed = -3400;
	 }
	 else
	 {
      CAN2_DATA.M_203.Tar_speed = PID_Update_Incre(&CAN2_DATA.M_203.PID_Postion,CAN2_DATA.M_203.Tar_Postion,CAN2_DATA.M_203.Cur_postion);
	 }
	 if(CAN2_DATA.M_204.Tar_Postion - CAN2_DATA.M_204.Cur_postion > 0.5)
	 {
	    CAN2_DATA.M_204.Tar_speed = 3400;
	 }
	 else if(CAN2_DATA.M_204.Tar_Postion - CAN2_DATA.M_204.Cur_postion < -0.5)
	 {
	    CAN2_DATA.M_204.Tar_speed = -3400;
	 }
	 else
	 {
      CAN2_DATA.M_204.Tar_speed = PID_Update_Incre(&CAN2_DATA.M_204.PID_Postion,CAN2_DATA.M_204.Tar_Postion,CAN2_DATA.M_204.Cur_postion);
	 }	 
	 CAN2_DATA.M_203.speed_out = PID_Update_Incre(&CAN2_DATA.M_203.PID_Speed,CAN2_DATA.M_203.Tar_speed,CAN2_DATA.M_203.Cur_speed);
	 CAN2_DATA.M_204.speed_out = PID_Update_Incre(&CAN2_DATA.M_204.PID_Speed,CAN2_DATA.M_204.Tar_speed,CAN2_DATA.M_204.Cur_speed);
}

void Can_out()
{
   CanTxMsg	canmsg;
	 can_data[0] = (u8)(CAN2_DATA.M_201.speed_out>>8);
	 can_data[1] = (u8)(CAN2_DATA.M_201.speed_out);
	 can_data[2] = (u8)(CAN2_DATA.M_202.speed_out>>8);
	 can_data[3] = (u8)(CAN2_DATA.M_202.speed_out);
	 can_data[4] = (u8)(CAN2_DATA.M_203.speed_out>>8);
	 can_data[5] = (u8)(CAN2_DATA.M_203.speed_out);
	 can_data[6] = (u8)(CAN2_DATA.M_204.speed_out>>8);
	 can_data[7] = (u8)(CAN2_DATA.M_204.speed_out);
	 can_set(&canmsg, 0x200,can_data);	
	 CAN_Transmit(CAN2,&canmsg);	
   
	 can_data[0] = (u8)(CAN1_DATA.M_205.speed_out>>8);
	 can_data[1] = (u8)(CAN1_DATA.M_205.speed_out);
	 can_data[2] = (u8)(CAN1_DATA.M_206.speed_out>>8);
	 can_data[3] = (u8)(CAN1_DATA.M_206.speed_out);
	 can_data[4] = (u8)(CAN1_DATA.M_207.speed_out>>8);
	 can_data[5] = (u8)(CAN1_DATA.M_207.speed_out);
	 can_data[6] = (u8)(CAN1_DATA.M_208.speed_out>>8);
	 can_data[7] = (u8)(CAN1_DATA.M_208.speed_out);
	 can_set(&canmsg, 0x1ff,can_data);	
	 CAN_Transmit(CAN1,&canmsg);	
}
   
   volatile u32 heat_sen,speed_sen;//哨兵17枪口热量,速度
	 volatile int pullnumber,pulls;//可发子弹数,已经打出的子弹数
	 //judge_recv_mesg.ext_power_heat_data.shooter_heat0;//裁判系统的热量
	 //judge_recv_mesg.ext_shoot_data.bullet_speed;

void pull_ceshi(void *param)
{
	 while(1)
	 {
		  if(heat_sen<300)//全速射击
	    {
		     CAN2_DATA.M_203.Tar_Postion -= 4; //顺时针转动
		     CAN2_DATA.M_204.Tar_Postion += 4;
				 if(CAN2_DATA.M_203.Cur_speed != 0)
				    heat_sen += 25;
	    }
			if(heat_sen>=300)
			{
				 CAN2_DATA.M_203.Tar_Postion -= 4; //顺时针转动
		     CAN2_DATA.M_204.Tar_Postion += 4;
				 if(CAN2_DATA.M_203.Cur_speed != 0)
				    heat_sen += 25;
				 task_delay_ms(500);
			}
			task_delay_ms(1);
	 }

}

void heat_ceshi(void *param)
{
	 while(1)
	 {
		  heat_sen -= 10;
		  task_delay_ms(100);
	 }
}

void sentry_pull(void *param)
{
   Init_pull();
	 while(1)
	 {
	    Updata_pos();//遥控器控制拨弹
		  Updata_out();
		  Can_out();
		  task_delay_ms(1);
		  print_wave(2,4,&CAN2_DATA.M_203.Tar_Postion,&CAN2_DATA.M_203.Cur_postion);
	 }
}



