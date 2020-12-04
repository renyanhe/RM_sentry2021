#ifndef _USER_CAN_H
#define _USER_CAN_H

#include "makos_includes.h"
#include "timer.h"
#include "headfile.h"
#include "pid.h"

typedef	struct	_mcircle_t
{
	volatile S32	circle;
	volatile S16	angle;
	volatile float Cur_postion;
	volatile int16_t Cur_speed;
	volatile int16_t Cur_I;
	int16_t Tar_speed;//Ŀ���ٶ�
	float Tar_Postion;//Ŀ��λ��
	int16_t Tar_I;//Ŀ�����
	PID_IncreType PID_Speed;
	PID_IncreType PID_Postion;//λ�û�
	PID_IncreType PID_I;//������
	int16_t speed_out;
}mcircle_t, *p_mcircle;

typedef struct _can_t
{
   mcircle_t M_201;
	 mcircle_t M_202;
	 mcircle_t M_203;
	 mcircle_t M_204;
	 mcircle_t M_205;
	 mcircle_t M_206;
	 mcircle_t M_207;
	 mcircle_t M_208;
} Typedef_can;
extern Typedef_can CAN1_DATA;
extern Typedef_can CAN2_DATA;

extern u8 can_data[16];
#endif

