#include "sentry_holder.h"

/*
�ϸ���		0x208 pitch_2
��ƫ��		0x207 yaw_2
�¸���		0x206 pitch
��ƫ��		0x205 yaw
*/
/*
���ص��Σ�CAN1_DATA.M_205.PID_Speed ,20,0.5,2,10000
          CAN1_DATA.M_205.PID_Postion ,5000,10,20,10000
���أ�
��ƫ��		0x207 yaw
					CAN1_DATA.M_207.PID_Speed ,20,0.02,5,10000
          CAN1_DATA.M_207.PID_Postion ,20000,50,100,10000
										
�ϸ���		0x208 pitch:��λ��0.93����1.11
					CAN1_DATA.M_208.PID_Speed ,15,0.1,1,10000
          CAN1_DATA.M_208.PID_Postion ,25000,50,10,10000
					
��ƫ��		0x205 yaw_2
					
�¸���		0x206 pitch_2:��λ�� -0.06����0.15
*/
/*�޸��������̨����������ֿ����ֱ������󣬺��ڲ��Կ����ֱ���pid�㷨*/
	
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
char flag_holder;//����λ
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
	 
	 if(flag_R == RP_S_DOWN)//�Զ�
	 {
		 flag_holder = 1;
		 if(dist_new2020==0)//Ѳ������û����
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

			 CAN1_DATA.M_205.Tar_Postion = CAN1_DATA.M_205.Cur_postion;//����λ��
			 CAN1_DATA.M_206.Tar_Postion = CAN1_DATA.M_206.Cur_postion;
			 CAN1_DATA.M_207.Tar_Postion = CAN1_DATA.M_207.Cur_postion;
			 CAN1_DATA.M_208.Tar_Postion = CAN1_DATA.M_208.Cur_postion;
		 }
		 else
		 {
			 CAN1_DATA.M_205.Tar_speed = PID_vision(&CAN1_DATA.M_205.PID_Postion,-yaw_angle2021);
			 CAN1_DATA.M_205.Tar_Postion = CAN1_DATA.M_205.Cur_postion - yaw_angle2021;//����λ��
		 }   
		// int i = 10;
	 }
	 if(flag_R == RP_S_MID)//�󲦸���Ϊͬ�����ƣ���ҡ�˵��������£����������ͬʱ����
	 {
		 flag_holder = 0;
		  if(i>0)//����ԭλ��
			{
				CAN1_DATA.M_205.Tar_Postion = CAN1_DATA.M_205.Cur_postion;
				CAN1_DATA.M_206.Tar_Postion = CAN1_DATA.M_206.Cur_postion;
				CAN1_DATA.M_207.Tar_Postion = CAN1_DATA.M_207.Cur_postion;
				CAN1_DATA.M_208.Tar_Postion = CAN1_DATA.M_208.Cur_postion;
				i--;
			}
			else
			{
				CAN1_DATA.M_205.Tar_Postion += (float)remote_yaw/330000;//ע�������������㷽��
				CAN1_DATA.M_207.Tar_Postion += (float)remote_yaw/330000;
				CAN1_DATA.M_206.Tar_Postion += (float)remote_pitch/330000;
				if(CAN1_DATA.M_206.Tar_Postion>=0.15)    CAN1_DATA.M_206.Tar_Postion = 0.15;
				if(CAN1_DATA.M_206.Tar_Postion <= -0.06) CAN1_DATA.M_206.Tar_Postion = -0.06;
				CAN1_DATA.M_208.Tar_Postion += (float)remote_pitch/330000;
				if(CAN1_DATA.M_208.Tar_Postion>=1.11)   CAN1_DATA.M_208.Tar_Postion = 1.11;
				if(CAN1_DATA.M_208.Tar_Postion <= 0.93) CAN1_DATA.M_208.Tar_Postion = 0.93;
			}
	 }
	 if(flag_R == RP_S_UP)//�ֿ����ƣ����̴˴α����Զ���ֹͣ
	 {
		 flag_holder = 0;
			if(i>0)//����ԭλ��
			{
				CAN1_DATA.M_205.Tar_Postion = CAN1_DATA.M_205.Cur_postion;
				CAN1_DATA.M_206.Tar_Postion = CAN1_DATA.M_206.Cur_postion;
				CAN1_DATA.M_207.Tar_Postion = CAN1_DATA.M_207.Cur_postion;
				CAN1_DATA.M_208.Tar_Postion = CAN1_DATA.M_208.Cur_postion;
				i--;
			}
			else
			{
				CAN1_DATA.M_205.Tar_Postion += (float)remote_yaw/330000;//ע�������������㷽��
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

void sentry_holder(void *param)     //��̨����
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
