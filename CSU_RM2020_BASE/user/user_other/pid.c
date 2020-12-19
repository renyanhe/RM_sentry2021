/*
	FILE NAME:	pid and calman.c
	DESCRIPTION:	the task dispose remote messages by the control rules
			and send the result to the relatively tasks
	
	EDIT RECORDS:
	---------
	AUTHOR:		the Seniors 
	EDIT TIME:	2017/11/18-13:47
	MODIFICATION:	built the frame of remote task, simply send the msg to chassis and holder.
	---------
	AUTHOR:		
	EDIT TIME:	
	MODIFICATION:	
	---------
*/
#include "pid.h"
//******************************************

//PID更新，输入参数：PID结构体，目标值，当前值；
//返回值为PID输出
float PID_Update_Incre(PID_IncreType* PID,float tar,float cur)
{
	float ep,ei,ed;

	PID->e0 =tar - cur;  //读取现在的误差，用于kp控制
	
	ep = PID->e0 - PID->e1;
	ei = PID->e0;
	ed = PID->e0 - 2*PID->e1 + PID->e2;

	PID->out += PID->kp*ep + PID->ki*ei + PID->kd*ed;
	
	PID->e2 = PID->e1;
	PID->e1 = PID->e0;	
	
	//PID->out = range(PID->out, -PID->limit, PID->limit);//增量限幅
	
	return PID->out;
}

float PID_vision(PID_IncreType* PID,float err)  //哨兵视觉PID
{
	float ep,ei,ed;

	PID->e0 =err;  //读取现在的误差，用于kp控制
	
	ep = PID->e0 - PID->e1;
	ei = PID->e0;
	ed = PID->e0 - 2*PID->e1 + PID->e2;
	
	PID->out += PID->kp*ep + PID->ki*ei + PID->kd*ed;
	
	PID->e2 = PID->e1;
	PID->e1 = PID->e0;	
	
	return PID->out;
}

float PID_Update_Iq(PID_IncreType* PID,float tar,float cur) //抗积分饱和pid
{
	float Speed_OutPreSat, Up, Ui, Ud;

	PID->e0 = tar - cur;  //读取现在的误差，用于kp控制

	Up = PID->kp*PID->e0;//比例部分
	Ui = Ui + PID->ki*Up + PID->ki*PID->e2;//积分部分
	Ud = PID->kd*(PID->e0 - PID->e1);

	Speed_OutPreSat = Up + Ui + Ud;
  PID->out = range(Speed_OutPreSat, -PID->limit, PID->limit);
	PID->e2 = PID->out - Speed_OutPreSat;
	PID->e1 = PID->e0;
	
	return PID->out;
}

float PID_Update_Absolute2(PID_AbsoluteType* PID,float tar,float cur)
{
	float pe, ie, de;
	float out=0;
	
	PID->e0 = tar - cur;

	PID->eSum += PID->e0;
	
	de = PID->e0 - PID->e1;

	pe = PID->e0;
	ie = PID->eSum;

	PID->e1 = PID->e0;
	
//	if((tar < 25) && (tar > -25))
//	{
//		PID->eSum = 0;
//	}
	
	out = pe*(PID->Kp) + ie*(PID->Ki) + de*(PID->Kd);
	
	out = range(out, -PID->limit, PID->limit);
	return out;
}

float PID_Update_Absolute3(PID_AbsoluteType* PID,float tar,float cur)    
{
	float pe, ie, de;
	float out=0;
	
	PID->e0 = tar - cur;

	PID->eSum += PID->e0;
	
	PID->eSum = range(PID->eSum, -VUM_LIMIT, VUM_LIMIT);    //积分限幅
	
	de = PID->e0 - PID->e1;

	pe = PID->e0;
	ie = PID->eSum;

	PID->e1 = PID->e0;
	
  out = pe*(PID->Kp) + ie*(PID->Ki) + de*(PID->Kd);
	
	out = range(out, -PID->limit, PID->limit);
	return out;
}

float PID_Update_Absolute4(PID_AbsoluteType* PID,float tar,float cur)
{
	float pe, ie, de;
	float out=0;
	
	PID->e0 = tar - cur;

	PID->eSum += PID->e0;
	
  PID->eSum = range(PID->eSum, -SUM_LIMIT, SUM_LIMIT);
    
	de = PID->e0 - PID->e1;

	pe = PID->e0;
	ie = PID->eSum;

	PID->e1 = PID->e0;
	
    out = pe*(PID->Kp) + ie*(PID->Ki) + de*(PID->Kd);
	
    
    if(PID->e0 > 1500)                    
       out =  PID->limit;              
    else if(PID->e0 < -1500)           
       out =  -PID->limit;             
    
	out = range(out, -PID->limit, PID->limit);
	return out;
}

void PID_Struct_Init(PID_IncreType* PID,float kp,float ki,float kd,float out_max)
{
	PID->kp = kp;//比例
	PID->ki = ki;//积分
	PID->kd = kd;//微分
	PID->e0 = 0;
	PID->e1 = 0;
	PID->e2 = 0;
	PID->limit = out_max;
	PID->out = 0;
}

void PID_Struct_Init_abs(PID_AbsoluteType* PID,float kp,float ki,float kd,float out_max)
{
	PID->Kp = kp;//比例
	PID->Ki = ki;//积分
	PID->Kd = kd;//微分
	PID->e0 = 0;
	PID->e1 = 0;
	PID->eSum = 0;
	PID->limit = out_max;
}
void PID_Struct_Init_abs_nimi(PID_AbsoluteType* PID,float kp,float ki,float kd,float out_max)
{
	PID->Kp = kp;//比例
	PID->Ki = ki;//积分
	PID->Kd = kd;//微分
	PID->limit = out_max;
}


/*******************************************************************************************************/

