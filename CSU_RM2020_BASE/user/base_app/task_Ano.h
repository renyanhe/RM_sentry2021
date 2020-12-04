 #ifndef _ANO_H_
 #define _ANO_H_
 #include "usart.h"
 #include "makos_includes.h"
 #include "headfile.h"
 typedef struct
{
    u8 msg_id;
    u8 msg_data;
    u8 send_check;
    u8 send_pid1;
    u8 send_pid2;
    u8 send_pid3;
    u8 send_pid4;
    u8 send_pid5;
    u8 send_pid6;
	  u8 send_speed;
	  u8 send_motor;
	  u8 send_data1;
} dt_flag_t;

 typedef __packed struct
{
	U16 kp;
	U16 ki;
	U16 kd;
} ANO_pid_data_t;

 typedef __packed struct
{
	ANO_pid_data_t pid1;
	ANO_pid_data_t pid2;
	ANO_pid_data_t pid3;
	ANO_pid_data_t pid4;
	ANO_pid_data_t pid5;
	ANO_pid_data_t pid6;
	ANO_pid_data_t pid7;
	ANO_pid_data_t pid8;
	ANO_pid_data_t pid9;
	ANO_pid_data_t pid10;
	ANO_pid_data_t pid11;
	ANO_pid_data_t pid12;
	ANO_pid_data_t pid13;
	ANO_pid_data_t pid14;
	ANO_pid_data_t pid15;
	ANO_pid_data_t pid16;
	ANO_pid_data_t pid17;
	ANO_pid_data_t pid18;
	
} ANO_data_t;
extern ANO_data_t ANO_data;
 
void ANO_DT_Data_Exchange(void);
void ANO_DT_Send_PID(u8 group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p, float p3_i, float p3_d);
void ANO_DT_Data_Receive_Prepare(u8 data);
void ANO_DT_Data_Receive_Anl_Task(void);
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);
void task_ANO( void* param);
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void ANO_DT_Send_DATA(u8 group,int _t,int data_1,int data_2,int data_3,int data_4);
#endif



