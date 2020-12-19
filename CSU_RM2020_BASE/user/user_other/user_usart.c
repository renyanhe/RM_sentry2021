#include "user_usart.h"
#include "irq_dead_check.h"
#include "usart.h"
#include "can.h"
#include "user_can.h"
#include "task_holder.h"
#include "makos_print.h"
#include "makos_resolver.h"
#include "task_nimin.h"
#include "data_fifo.h"
#include "communicate_nimin.h"
#include "nimin_info.h"
#include "stdarg.h"
#include "task_vision.h"

extern u8 VS_Rx_Pos;                           // 用于记录接收缓存的位置（视觉通信）
//extern u8 VS_rx_Msg[vs_rxbuf_len];           //存储 PC->Robot 视觉信息
u8 VS_rx_Msg[8];
extern u8 visual_Enable;

IRQ_CHECK_DEFINE(usart1);
void	USART1_IRQHandler(void)
{
	IRQ_CHECK(usart1);
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		uif1();
	}
}

/*u8 TxCounter = 0;
void	USART2_IRQHandler(void)
{
	u8 com_data;
	if ( USART2->SR & USART_SR_ORE ) //ORE中断（溢出中断）
    {
        com_data = USART2->DR;
    }

    //接收中断
    if ( USART_GetITStatus ( USART2, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( USART2, USART_IT_RXNE ); //清除中断标志

        com_data = USART2->DR;
		//	printf("data = %x\n",com_data);
        ANO_DT_Data_Receive_Prepare ( com_data ); //接收处理
    }
    //发送（进入移位）中断
    if ( USART_GetITStatus ( USART2, USART_IT_TXE ) )
    {

        USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志
        if ( TxCounter == count )
        {
            USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
        }
    }
}*/

/*IRQ_CHECK_DEFINE(usart4);
void	UART4_IRQHandler(void)
{
	IRQ_CHECK(usart4);
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		uif4();
	}
}*/

IRQ_CHECK_DEFINE(usart5);
void	UART5_IRQHandler(void)
{
	IRQ_CHECK(usart5);
	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);

		uif5();
	}
}


uint8_t Camera_Uart_Buffer[256];
u8 Target_Flag,Target_Flag_last;
S16 camera_num = 0;

static float x_tar_base,x_tar_base_last,holder_x_target2;
//static float y_tar_base,y_tar_base_last;
static float vision_a,v_last;
float a_moment_x,a_moment_y;
float vision_a_k = 4.4;
float vision_a_k_pitch = 5000;
static float k = 0.250;
static float kf_q3 = 0.3,kf_r3 = 1;

u8 flag_fass2 = 1;

u8 flag_pass_x_base,flag_pass_y_base;
#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))//a-最小值   b-最大值


typedef union
{
	unsigned char bit[4];
	float a;
} rec_data_e;			//用于接收视觉float信息的联合体

rec_data_e camera_new[3];volatile float yaw_new2020,pitch_new2020,dist_new2020,yaw_angle2021,pitch_angle2021;
volatile u8 get_flag = 0;
IRQ_CHECK_DEFINE(usart6);

float volatile __count = 0;
void	USART6_IRQHandler(void)
{
	static u8 count = 0;
	IRQ_CHECK(usart6);
	if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
		Camera_Uart_Buffer[count] = USART_ReceiveData(USART6);	//读取接收到的数据
		if(Camera_Uart_Buffer[count] == 0X03 && Camera_Uart_Buffer[count-1] == 0XFC)
		{
			if(Camera_Uart_Buffer[count - 15] == 0X03 && Camera_Uart_Buffer[count - 14] == 0XFC)		
			{
				camera_num ++;
				camera_new[0].bit[0] = Camera_Uart_Buffer[count - 13];
				camera_new[0].bit[1] = Camera_Uart_Buffer[count - 12];
				camera_new[0].bit[2] = Camera_Uart_Buffer[count - 11];
				camera_new[0].bit[3] = Camera_Uart_Buffer[count - 10];
				yaw_new2020 = camera_new[0].a;
				yaw_angle2021 = yaw_new2020/360;//处理为角度
				
				camera_new[1].bit[0] = Camera_Uart_Buffer[count - 9];
				camera_new[1].bit[1] = Camera_Uart_Buffer[count - 8];
				camera_new[1].bit[2] = Camera_Uart_Buffer[count - 7];
				camera_new[1].bit[3] = Camera_Uart_Buffer[count - 6];
				pitch_new2020 = camera_new[1].a;
				pitch_angle2021 = pitch_new2020/360;
				
				camera_new[2].bit[0] = Camera_Uart_Buffer[count - 5];
				camera_new[2].bit[1] = Camera_Uart_Buffer[count - 4];
				camera_new[2].bit[2] = Camera_Uart_Buffer[count - 3];
				camera_new[2].bit[3] = Camera_Uart_Buffer[count - 2];
				dist_new2020 = camera_new[2].a;
				__count++;
				}
		}
		count ++;
		if(count == 255)
			count = 0;
		
	}
}

void usart3_send_char(char ch)
{
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);
	USART_SendData(USART3,(u8)ch);	
}

void usart3_send_string(u8 *buff, u32 len)
{
    while(len--)
    {
        usart3_send_char(*buff);
        buff++;
    }
}


void usart6_send_char(char ch)
{
	while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
	USART_SendData(USART6,(u8)ch);	
}

void usart6_send_string(u8 *buff, u32 len)
{
    while(len--)
    {
        usart6_send_char(*buff);
        buff++;
    }
}

void usart2_send_char(char ch)
{
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	USART_SendData(USART2,(u8)ch);	
}

void usart2_send_string(u8 *buff, u32 len)
{
    while(len--)
    {
        usart2_send_char(*buff);
        buff++;
    }
}

#define CMD_WAVE 3	//山外上位机对应的波形指令
uint8_t cmdf[2] = {CMD_WAVE, ~CMD_WAVE};   
uint8_t cmdr[2] = {~CMD_WAVE, CMD_WAVE};
void usart6_send_wave_int16(u8* p,u8 num)
{
	u8 i;
	usart6_send_string(cmdf, sizeof(cmdf));    
			
	for(i = 0;i < num;i++)
	{
		usart6_send_string(p + 2 * i,2);
		//usart6_send_string((uint8_t *)&DataInfo[4],4);
	}		
	usart6_send_string(cmdr, sizeof(cmdr)); 
}

extern  S16	  get_bullet_shoot_speed;
extern 	S16		get_bullet_speed;
extern 	S16		get_bullet_temp;
extern  float	  get_bullet_circle;

extern  S16	  car_spin_engineer_shoot_speed;
extern 	S16		car_spin_engineer_speed;
extern 	S16		car_spin_engineer_temp;
extern  float	  car_spin_engineer_circle;

extern  S16	  car_spin_engineer_shoot_speedFF;
extern 	S16		car_spin_engineer_speedFF;
extern 	S16		car_spin_engineer_tempFF;
extern  float	  car_spin_engineer_circleFF;

extern  S16	  shoot_speed;
extern 	S16		speed;
extern 	S16		temp;
extern  S16	  circle;

//**************???????**************	


s16 SHOOT_CIRCLE_F,SHOOT_CIRCLE_FF,SHOOT_CIRCLE_G;

void send_wave()
{     
//	S16	pos_temp = SHOOT_CIRCLE_F;
//	S16	pos_tar_temp = car_spin_engineer_circle;
//	S16	pos_tempFF = SHOOT_CIRCLE_FF;
//	S16	pos_tar_tempFF = car_spin_engineer_circleFF;
//	
//	S16 pos_get_bullet_temp = SHOOT_CIRCLE_G;
//  S16 pos_get_bullet_tar_temp = get_bullet_circle;
//	
//    usart3_send_string(cmdf, sizeof(cmdf));    
//	
//	/***********取弹************/
//    usart3_send_string((uint8_t *)&get_bullet_shoot_speed,2);                   //速度环实际值
//		usart3_send_string((uint8_t *)&get_bullet_speed,2);                         //速度环输出值        
//		usart3_send_string((uint8_t *)&get_bullet_temp,2);                          //速度环目标值  位置环输出值  
//		
//		usart3_send_string((uint8_t *)&pos_get_bullet_tar_temp,2);                  //位置环目标值    
//		usart3_send_string((uint8_t *)&pos_get_bullet_temp,2);                      //位置环实际值
//	
	/*******上岛*******/
//    usart3_send_string((uint8_t *)&car_spin_engineer_shoot_speed,2);  //速度环实际值
//		usart3_send_string((uint8_t *)&car_spin_engineer_speed,2);        //速度环输出值        
//		usart3_send_string((uint8_t *)&car_spin_engineer_temp,2);         //速度环目标值  位置环输出值  
//		
//		usart3_send_string((uint8_t *)&pos_tar_temp,2);                  //位置环目标值    
//		usart3_send_string((uint8_t *)&pos_temp,2);                      //位置环实际值
		
//    usart3_send_string((uint8_t *)&car_spin_engineer_shoot_speedFF,2);  //速度环实际值
//		usart3_send_string((uint8_t *)&car_spin_engineer_speedFF,2);        //速度环输出值        
//		usart3_send_string((uint8_t *)&car_spin_engineer_tempFF,2);         //速度环目标值  位置环输出值  
//		
//		usart3_send_string((uint8_t *)&pos_tar_tempFF,2);                  //位置环目标值    
//		usart3_send_string((uint8_t *)&pos_tempFF,2);                      //位置环实际值
		
	/*****发弹*******/
//	  usart3_send_string((uint8_t *)&shoot_speed,2);  //速度环实际值
//		usart3_send_string((uint8_t *)&speed,2);        //速度环输出值
//		usart3_send_string((uint8_t *)&temp,2);         //速度环目标值  位置环输出值

//		usart3_send_string((uint8_t *)&circle,2);        //位置环目标值
//		usart3_send_string((uint8_t *)&SHOOT_CIRCLE,2);  //位置环实际值
		
    usart3_send_string(cmdr, sizeof(cmdr));    
		//***********************************
}

#define CMD_WARE     3
/**************
函数功能：向山外上位机输出1到8个通道的波形
形参说明：number：变量数目
		  length：变量大小，以字节为单位
备注：调用示例：print_wave(3,1,(uint8_t *)&send_date_1,(uint8_t *)&send_date_2,(uint8_t *)&send_date_3);//float是4字节
**************/
void print_wave(u8 number, u8 length, ...)
{
    va_list ap;
    u8 temp_number;
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};

    va_start(ap, length);

    usart2_send_string(cmdf, sizeof(cmdf));

    for(temp_number = 0; temp_number < number; temp_number++)
    {
        usart2_send_string(va_arg(ap, u8*), length);
    }

    usart2_send_string(cmdr, sizeof(cmdr));
    va_end(ap);
}

