#ifndef _HC_H_
#define _HC_H_
#include "headfile.h"

typedef struct _HC_
{
   volatile u8 sta;//输入捕获状态
   volatile u16 temp;	
	 volatile u16 Distance;
	 volatile u16 val;//输入捕获值
}HC;

extern HC HC_1;
extern HC HC_2;
#define TRIG1_Send  PBout(0) 
#define ECHO1_Reci  PBin(1)	 
#define TRIG2_Send	PEout(6) 
#define ECHO2_Reci	PEin(5)

void HC_Reci_TIM3_Init(u16 arr,u16 psc);
void HC_Reci_TIM9_Init(u16 arr,u16 psc);

void HC_ult_send(void);
#endif
