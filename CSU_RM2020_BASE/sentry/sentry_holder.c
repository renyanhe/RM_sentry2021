#include "sentry_holder.h"

/*
上俯仰		0x208 pitch_2
上偏航		0x207 yaw_2
下俯仰		0x206 pitch
下偏航		0x205 yaw
*/
/*
空载调参：CAN1_DATA.M_205.PID_Speed ,20,0.5,2,10000
          CAN1_DATA.M_205.PID_Postion ,5000,10,20,10000
带载：
上偏航		0x207 yaw
					CAN1_DATA.M_207.PID_Speed ,20,0.02,5,10000
          CAN1_DATA.M_207.PID_Postion ,20000,50,100,10000
										
上俯仰		0x208 pitch:限位：0.93——1.11
					CAN1_DATA.M_208.PID_Speed ,15,0.1,1,10000
          CAN1_DATA.M_208.PID_Postion ,25000,50,10,10000
					
下偏航		0x205 yaw_2
					
下俯仰		0x206 pitch_2:限位： -0.06——0.15
*/
/*修改意见：云台俯仰电机出现抗积分饱和现象，后期测试抗积分饱和pid算法*/
	
void Init_hlolder()
{
   PID_Struct_Init(&CAN1_DATA.M_205.PID_Speed ,20,0.02,5,10000);
	 PID_Struct_Init(&CAN1_DATA.M_205.PID_Postion ,20000,50,100,10000);
	
	 PID_Struct_Init(&CAN1_DATA.M_206.PID_Speed ,15,0.1,1,10000);
	 PID_Struct_Init(&CAN1_DATA.M_206.PID_Postion ,25000,50,10,10000);
	
	 PID_Struct_Init(&CAN1_DATA.M_207.PID_Speed ,20,0.02,5,10000);
	 PID_Struct_Init(&CAN1_DATA.M_207.PID_Postion ,20000,50,100,10000);
	
	 PID_Struct_Init(&CAN1_DATA.M_208.PID_Speed ,15,0.1,1,10000);
   PID_Struct_Init(&CAN1_DATA.M_208.PID_Postion ,25000,50,10,10000);
}

u8 flag_R = RP_S_MID;
int i = 15;
char flag_holder;//控制位
u8 dir=1;
int16_t remote_yaw,speed_o;
int16_t remote_yaw2;
int16_t remote_pitch;
int16_t remote_pitch2;
float cur_pos1,cur_pos2,cur_pos3,cur_pos4,tur_pos1,tur_pos2,tur_pos3,tur_pos4;
void Updata_Out()
{
   p_sentry sentry;
	 sentry = msg_get_read_try(&sentry_msg);
	 if(sentry != NULL)
	 {
			flag_R = sentry->SW_right;
			remote_yaw = sentry->yaw;
			remote_yaw2 =  sentry->yaw_2;
			remote_pitch = -sentry->pitch;
			remote_pitch2 = sentry->pitch_2;
	 }
	 msg_read_finish_try(&sentry_msg);
	 
	 if(flag_R == RP_S_DOWN)//自动
	 {
		 flag_holder = 1;
		 if(dist_new2020==0)//巡航部分没调完
		 {
			 CAN1_DATA.M_205.Tar_speed = 55;
//			 CAN1_DATA.M_206.Tar_Postion = 0.10;
			 if( CAN1_DATA.M_206.Cur_postion >= 0.14 )  dir = 0;
			 if( CAN1_DATA.M_206.Cur_postion <= -0.04 ) dir = 1;
			 if(dir) CAN1_DATA.M_206.Tar_speed = 25;
			 else CAN1_DATA.M_206.Tar_speed = -25;
			 
//			 CAN1_DATA.M_206.Tar_speed = PID_Update_Incre(&CAN1_DATA.M_206.PID_Postion,CAN1_DATA.M_206.Tar_Postion,CAN1_DATA.M_206.Cur_postion);
			 
			 CAN1_DATA.M_207.Tar_speed = 60;
			 CAN1_DATA.M_208.Tar_Postion = 1.10;
			 CAN1_DATA.M_208.Tar_speed = PID_Update_Incre(&CAN1_DATA.M_206.PID_Postion,CAN1_DATA.M_208.Tar_Postion,CAN1_DATA.M_208.Cur_postion);

			 CAN1_DATA.M_205.Tar_Postion = CAN1_DATA.M_205.Cur_postion;//矫正位置
			 CAN1_DATA.M_206.Tar_Postion = CAN1_DATA.M_206.Cur_postion;
			 CAN1_DATA.M_207.Tar_Postion = CAN1_DATA.M_207.Cur_postion;
			 CAN1_DATA.M_208.Tar_Postion = CAN1_DATA.M_208.Cur_postion;
		 }
		 else
		 {
			 CAN1_DATA.M_205.Tar_speed = PID_vision(&CAN1_DATA.M_205.PID_Postion,-yaw_angle2021);
			 CAN1_DATA.M_205.Tar_Postion = CAN1_DATA.M_205.Cur_postion - yaw_angle2021;//矫正位置
		 }   
		// int i = 10;
	 }
	 if(flag_R == RP_S_MID)//左拨杆中为同步控制：右摇杆的左右上下，可以与底盘同时测试
	 {
		 flag_holder = 0;
		  if(i>0)//保持原位置
			{
				CAN1_DATA.M_205.Tar_Postion = CAN1_DATA.M_205.Cur_postion;
				CAN1_DATA.M_206.Tar_Postion = CAN1_DATA.M_206.Cur_postion;
				CAN1_DATA.M_207.Tar_Postion = CAN1_DATA.M_207.Cur_postion;
				CAN1_DATA.M_208.Tar_Postion = CAN1_DATA.M_208.Cur_postion;
				i--;
			}
			else
			{
				CAN1_DATA.M_205.Tar_Postion += (float)remote_yaw/330000;//注意这里除数与计算方法
				CAN1_DATA.M_207.Tar_Postion += (float)remote_yaw/330000;
				CAN1_DATA.M_206.Tar_Postion += (float)remote_pitch/330000;
				if(CAN1_DATA.M_206.Tar_Postion>=0.15)    CAN1_DATA.M_206.Tar_Postion = 0.15;
				if(CAN1_DATA.M_206.Tar_Postion <= -0.06) CAN1_DATA.M_206.Tar_Postion = -0.06;
				CAN1_DATA.M_208.Tar_Postion += (float)remote_pitch/330000;
				if(CAN1_DATA.M_208.Tar_Postion>=1.11)   CAN1_DATA.M_208.Tar_Postion = 1.11;
				if(CAN1_DATA.M_208.Tar_Postion <= 0.93) CAN1_DATA.M_208.Tar_Postion = 0.93;
			}
	 }
	 if(flag_R == RP_S_UP)//分开控制；底盘此次必须自动或停止
	 {
		 flag_holder = 0;
			if(i>0)//保持原位置
			{
				CAN1_DATA.M_205.Tar_Postion = CAN1_DATA.M_205.Cur_postion;
				CAN1_DATA.M_206.Tar_Postion = CAN1_DATA.M_206.Cur_postion;
				CAN1_DATA.M_207.Tar_Postion = CAN1_DATA.M_207.Cur_postion;
				CAN1_DATA.M_208.Tar_Postion = CAN1_DATA.M_208.Cur_postion;
				i--;
			}
			else
			{
				CAN1_DATA.M_205.Tar_Postion += (float)remote_yaw/330000;//注意这里除数与计算方法
				CAN1_DATA.M_207.Tar_Postion += (float)remote_yaw2/330000;
				CAN1_DATA.M_206.Tar_Postion += (float)remote_pitch/330000;
				if(CAN1_DATA.M_206.Tar_Postion>=0.15)    CAN1_DATA.M_206.Tar_Postion = 0.15;
				if(CAN1_DATA.M_206.Tar_Postion <= -0.06) CAN1_DATA.M_206.Tar_Postion = -0.06;
				CAN1_DATA.M_208.Tar_Postion += (float)remote_pitch2/330000;
				if(CAN1_DATA.M_208.Tar_Postion>=1.11)   CAN1_DATA.M_208.Tar_Postion = 1.11;
				if(CAN1_DATA.M_208.Tar_Postion <= 0.93) CAN1_DATA.M_208.Tar_Postion = 0.93;
			}
	 } 
	 if(flag_holder == 0)
	 {
		 CAN1_DATA.M_205.Tar_speed = PID_Update_Incre(&CAN1_DATA.M_205.PID_Postion,CAN1_DATA.M_205.Tar_Postion,CAN1_DATA.M_205.Cur_postion);
		 CAN1_DATA.M_206.Tar_speed = PID_Update_Incre(&CAN1_DATA.M_206.PID_Postion,CAN1_DATA.M_206.Tar_Postion,CAN1_DATA.M_206.Cur_postion);
		 CAN1_DATA.M_207.Tar_speed = PID_Update_Incre(&CAN1_DATA.M_207.PID_Postion,CAN1_DATA.M_207.Tar_Postion,CAN1_DATA.M_207.Cur_postion);
		 CAN1_DATA.M_208.Tar_speed = PID_Update_Incre(&CAN1_DATA.M_208.PID_Postion,CAN1_DATA.M_208.Tar_Postion,CAN1_DATA.M_208.Cur_postion);
	 } 
	  CAN1_DATA.M_205.speed_out = PID_Update_Incre(&CAN1_DATA.M_205.PID_Speed,CAN1_DATA.M_205.Tar_speed,CAN1_DATA.M_205.Cur_speed);
	  CAN1_DATA.M_206.speed_out = PID_Update_Incre(&CAN1_DATA.M_206.PID_Speed,CAN1_DATA.M_206.Tar_speed,CAN1_DATA.M_206.Cur_speed);
//    CAN1_DATA.M_207.speed_out = PID_Update_Incre(&CAN1_DATA.M_207.PID_Speed,CAN1_DATA.M_207.Tar_speed,CAN1_DATA.M_207.Cur_speed);
//    CAN1_DATA.M_208.speed_out = PID_Update_Incre(&CAN1_DATA.M_208.PID_Speed,CAN1_DATA.M_208.Tar_speed,CAN1_DATA.M_208.Cur_speed);
															 
	 tur_pos1 = CAN1_DATA.M_205.Tar_Postion;
	 tur_pos2 = CAN1_DATA.M_206.Tar_Postion;
	 tur_pos3 = CAN1_DATA.M_207.Tar_Postion;
	 tur_pos4 = CAN1_DATA.M_208.Tar_Postion;
	 cur_pos1 = CAN1_DATA.M_205.Cur_postion;
	 cur_pos2 = CAN1_DATA.M_206.Cur_postion;
	 cur_pos3 = CAN1_DATA.M_207.Cur_postion;
	 cur_pos4 = CAN1_DATA.M_208.Cur_postion;
	 speed_o = CAN1_DATA.M_206.speed_out;
}

void sentry_holder(void *param)     //云台任务
{
	Init_hlolder();
	 while(1)
	 {
      Updata_Out();
		  Can_out();
//			print_wave(1,2,&remote_yaw);
//		  print_wave(3,2,&yaw_new2020,&pitch_new2020,&dist_new2020);
		  task_delay_ms(1);
	 }
}
