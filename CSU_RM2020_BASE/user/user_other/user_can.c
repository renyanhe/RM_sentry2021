#include "user_can.h"
#include "can.h"
#include "user_can.h"
#include "task_holder.h"
#include "motor.h"
#include "flash_data.h"
#include "timer.h"

void	CAN1_TX_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET)
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
		canrate.inc.can1_tx++;
	}
}

void	CAN2_TX_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN2,CAN_IT_TME) != RESET)
	{		
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
		canrate.inc.can2_tx++;
	}
}

#define	CIRCLE_FASTEST	1000

void	circle_count(p_mcircle pmc, U16 angle, S16 speed)
{
	if(	(angle < CIRCLE_FASTEST) && (pmc->angle > 8192-CIRCLE_FASTEST) && (speed > 0) )
	{
		pmc->circle++;
	}
	else if((angle > 8192-CIRCLE_FASTEST) && (pmc->angle < CIRCLE_FASTEST) && (speed < 0) )
	{
		pmc->circle--;
	}
	pmc->angle = angle;
	pmc->Cur_postion = pmc->circle + (float)pmc->angle/8192;
}

void	circle_count_simple(p_mcircle pmc, U16 angle)
{
	if(	(angle < CIRCLE_FASTEST) && (pmc->angle > 8192-CIRCLE_FASTEST) )
	{
		pmc->circle++;
	}
	else if((angle > 8192-CIRCLE_FASTEST) && (pmc->angle < CIRCLE_FASTEST) )
	{
		pmc->circle--;
	}
	pmc->angle = angle;
}
Typedef_can CAN1_DATA;
Typedef_can CAN2_DATA;
u8 can_data[16];
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg	RxMessage;
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);		
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

		if(RxMessage.StdId == 0x205)
		 {	
        CAN1_DATA.M_205.Cur_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
			  circle_count(&CAN1_DATA.M_205,RxMessage.Data[0]<<8|RxMessage.Data[1],CAN1_DATA.M_205.Cur_speed);
		 }
		if(RxMessage.StdId == 0x206)
		 {	
        CAN1_DATA.M_206.Cur_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
			  circle_count(&CAN1_DATA.M_206,RxMessage.Data[0]<<8|RxMessage.Data[1],CAN1_DATA.M_206.Cur_speed);
		 }
		 if(RxMessage.StdId == 0x207)
		 {	
        CAN1_DATA.M_207.Cur_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
			  circle_count(&CAN1_DATA.M_207,RxMessage.Data[0]<<8|RxMessage.Data[1],CAN1_DATA.M_207.Cur_speed);
		 }
		 if(RxMessage.StdId == 0x208)
		 {	
        CAN1_DATA.M_208.Cur_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
			  circle_count(&CAN1_DATA.M_208,RxMessage.Data[0]<<8|RxMessage.Data[1],CAN1_DATA.M_208.Cur_speed);
		 }
	}
}


void	CAN2_RX0_IRQHandler(void)
{
	size_t		i;
	CanRxMsg	RxMessage;
	int16_t Cur_I1,Cur_I2,Cur_speed1,Cur_speed2 ;

	if(CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_ClearFlag(CAN2, CAN_FLAG_FMP0); 		
		CAN_Receive(CAN2,CAN_FIFO0,&RxMessage);		
		
			if(RxMessage.StdId == 0x201)
		 {	
			  Cur_speed1 = RxMessage.Data[2]<<8|RxMessage.Data[3];
			  CAN2_DATA.M_201.Cur_speed = LPF(0.7,Cur_speed1,CAN2_DATA.M_201.Cur_speed);
			  Cur_I1 = RxMessage.Data[4]<<8|RxMessage.Data[5];
			  CAN2_DATA.M_201.Cur_I = LPF(0.7,Cur_I1,CAN2_DATA.M_201.Cur_I);
			  circle_count(&CAN2_DATA.M_201,RxMessage.Data[0]<<8|RxMessage.Data[1],CAN2_DATA.M_201.Cur_speed);
		 }
		if(RxMessage.StdId == 0x202)
		 {	
        Cur_speed2 = RxMessage.Data[2]<<8|RxMessage.Data[3];
			  CAN2_DATA.M_202.Cur_speed = LPF(0.7,Cur_speed2,CAN2_DATA.M_202.Cur_speed);
			  Cur_I2 = RxMessage.Data[4]<<8|RxMessage.Data[5];
			  CAN2_DATA.M_202.Cur_I = LPF(0.7,Cur_I2,CAN2_DATA.M_202.Cur_I);
			  circle_count(&CAN2_DATA.M_202,RxMessage.Data[0]<<8|RxMessage.Data[1],CAN2_DATA.M_202.Cur_speed);
		 }
		if(RxMessage.StdId == 0x203)
		 {	
        CAN2_DATA.M_203.Cur_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
			  circle_count(&CAN2_DATA.M_203,RxMessage.Data[0]<<8|RxMessage.Data[1],CAN2_DATA.M_203.Cur_speed);
		 }
		if(RxMessage.StdId == 0x204)
		 {	
        CAN2_DATA.M_204.Cur_speed = RxMessage.Data[2]<<8|RxMessage.Data[3];
			  circle_count(&CAN2_DATA.M_204,RxMessage.Data[0]<<8|RxMessage.Data[1],CAN2_DATA.M_204.Cur_speed);
		 }
	
	}
}

