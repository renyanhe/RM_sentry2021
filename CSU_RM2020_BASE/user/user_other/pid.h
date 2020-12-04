#ifndef _PID_H
#define _PID_H

#include "makos_includes.h"

#define KF_Q 10
#define KF_R 10

#define SUM_LIMIT   400
#define VUM_LIMIT   3000

#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))//a-最小值   b-最大值

typedef struct 
{
	float limit;
	float kp;
	float ki;
	float kd;
	float e0;
	float e1;
	float e2;
	float out;
} PID_IncreType;//增量式PID

typedef struct 
{
	float limit;	//????
	float Kp;		
	float Ki;
	float Kd;
	float eSum;
	float e0;		
	float e1;		
}PID_AbsoluteType;//位置式PID


float PID_Update_Incre(PID_IncreType* PID,float tar,float cur);
float PID_Update_Absolute2(PID_AbsoluteType* PID,float tar,float cur);
float PID_Update_Absolute3(PID_AbsoluteType* PID,float tar,float cur);
float PID_Update_Absolute4(PID_AbsoluteType* PID,float tar,float cur);
void PID_Struct_Init(PID_IncreType* PID,float kp,float ki,float kd,float out_max);

void PID_Struct_Init_abs(PID_AbsoluteType* PID,float kp,float ki,float kd,float out_max);
void PID_Struct_Init_abs_nimi(PID_AbsoluteType* PID,float kp,float ki,float kd,float out_max);
float PID_Vision(PID_AbsoluteType* PID,float err);
float PID_vision(PID_IncreType* PID,float err);
#endif
