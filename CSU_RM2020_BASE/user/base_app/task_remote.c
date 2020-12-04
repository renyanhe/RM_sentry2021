/*
	FILE NAME:	task_remote.c
	DESCRIPTION:	the task dispose remote messages by the control rules
			and send the result to the relatively tasks
	
	EDIT RECORDS:
	---------
	AUTHOR:		FAN YAOWEN
	EDIT TIME:	2017/11/6-21:25
	MODIFICATION:	built the frame of remote task, simply send the msg to chassis and holder.
	---------
	AUTHOR:		FAN YAOWEN
	EDIT TIME:	2018/3/30
	MODIFICATION:	rewrote the shit code of receiver, add and change the romoting logic.
	---------
*/
#include "new_remote.h"
#include "task_remote.h"
#include "task_init.h"
#include "dbus.h"
#include "task_holder.h"
#include "timer.h"
#include "mak_filter.h"

msg_t	chassis_msg;
msg_t	holder_msg;
msg_t	shoot_msg;

msg_t	remote_msg;

msg_t sentry_msg;							//定义哨兵数据包

unsigned char magazine = 0;

void	remote_send_msg_init(void)
{
	msg_init(&remote_msg,	1,	sizeof(remote_data_t));
	msg_init(&sentry_msg,	1,	sizeof(sentry_t));           //哨兵   
}


void	remote_data_dispose(p_remote_data tg)
{
	tg->JR_LR	= (U16)((remote_buff[0]		| (remote_buff[1]<<8))				& 0x07ff);
	tg->JR_UD	= (U16)(((remote_buff[1]>>3)	| (remote_buff[2]<<5))				& 0x07ff);
	tg->JL_LR	= (U16)(((remote_buff[2]>>6)	| (remote_buff[3]<<2)	| (remote_buff[4]<<10))	& 0x07ff);
	tg->JL_UD	= (U16)(((remote_buff[4]>>1)	| (remote_buff[5]<<7))				& 0x07ff);
	tg->SL		= (U8)	((remote_buff[5]>>6)							& 0x03);
	tg->SR		= (U8)	((remote_buff[5]>>4)							& 0x03);
	
	tg->LL		= (U16)((remote_buff[16] | (remote_buff[17]<<8))&0x07FF);
	
	tg->MX		= (U16)(remote_buff[6]		| (remote_buff[7]<<8));
	tg->MY		= (U16)(remote_buff[8]		| (remote_buff[9]<<8));
	tg->MCL		= (U8)	remote_buff[12];
	tg->MCR		= (U8)	remote_buff[13];
	
	tg->KEY		= (U16)(remote_buff[14]		| (remote_buff[15]<<8));
	
	tg->KEY_data.Key_W=_K_CHK(W);
	tg->KEY_data.Key_A=_K_CHK(A);
	tg->KEY_data.Key_S=_K_CHK(S);
	tg->KEY_data.Key_D=_K_CHK(D);
	tg->KEY_data.Key_Q=_K_CHK(Q);
	tg->KEY_data.Key_E=_K_CHK(E);
	tg->KEY_data.Key_SHIFT=_K_CHK(SHIFT);
	tg->KEY_data.Key_CTRL=_K_CHK(CTRL);
	tg->KEY_data.Key_R=_K_CHK(R);
	tg->KEY_data.Key_F=_K_CHK(F);
	tg->KEY_data.Key_G=_K_CHK(G);
	tg->KEY_data.Key_Z=_K_CHK(Z);
	tg->KEY_data.Key_X=_K_CHK(X);
	tg->KEY_data.Key_C=_K_CHK(C);
	tg->KEY_data.Key_V=_K_CHK(V);
	tg->KEY_data.Key_B=_K_CHK(B);
}

void	remote_handle(void)
{
	irq_close();
	if(dbus_buff_checked())
	{
		p_remote_data	data;
		
		data = msg_get_write_some(&remote_msg);
		remote_data_dispose(data);
		msg_write_finish_some(&remote_msg);
	}
	irq_restore();
}

static
void	memcopy(void* target, void* source, U32 size)
{
	U32	i;
	U8	*pt = target;
	U8	*ps = source;
	for(i=0; i<size; i++){
		*pt	= *ps;
		pt++;
		ps++;
	}
}

void sentry_ctrl(p_remote_data	data)           //哨兵遥控器数据
{
   p_sentry sentry;
	 static int temp = 1024;	
	 sentry = msg_get_write_try(&sentry_msg);
	 if(sentry != NULL)
	 {
		  //云台数据:由右拨杆控制同步与否
		  sentry->yaw = (int)data->JR_LR - 1024;	
		  sentry->pitch = (int)data->JR_UD - 1024;
		  sentry->yaw_2 = (int)data->JL_LR - 1024;
		  sentry->pitch_2 = (int)data->JL_UD - 1024;
		  //2006拨弹逻辑:左小齿轮
		  if(data->LL <= 1320 && temp >= 1340)
			{
		     sentry->SL_pull = SHOOT;
			}
			else if(data->LL >= 800 && temp <= 780)
			{
			   sentry->SL_pull = SHOOT_2;
			}
			else
				sentry->SL_pull = UNSHOOT;
			 temp = data->LL;
			 //底盘控制数据：速度与运动控制形式
			 sentry ->SW_left = data->SL;
			 sentry ->SW_right = data->SR;
			 sentry->speed = (int)data->JL_LR - 1024;
		  msg_write_finish(&sentry_msg);
	 }
}



remote_data_t	task_remote_buff;
void	task_remote(void* param)
{
	static uint16_t remote_connect = 0;
	p_remote_data	data;
	dbus_init(1, remote_handle);
	
	while(1)
	{	
		data	= msg_get_read_some(&remote_msg);
		if(data)
		{
			irq_close();
			
			memcopy(&task_remote_buff, data, sizeof(remote_data_t));
			msg_read_finish_some(&remote_msg);
			
			sentry_ctrl(&task_remote_buff);
			
			irq_restore();
			remote_connect = 0;
		}
		else
		{
			if(remote_connect++ == 1500)
			{
				remote_connect = 0;
				dbus_init(1, remote_handle);
			}
		}
		task_delay_ms(5);
	}
}


