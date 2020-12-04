csu fyt机器人战队的各机器人间通用代码部分

资源使用情况：
tim1：小摩擦轮pwm输出
tim11:微妙延时用
tim12：中断速率检测用
uart1:遥控器（dma1）
uart2:调试用
uart3:裁判系统（dma2）
uart4:hi216陀螺仪
uart6:视觉通讯
iic1:oled
spi1:mpu6500陀螺仪

更新记录：
2019.8.31 
1.创建此仓库，相较之前的程序加入了task_mems，即单轴陀螺仪程序
2.加入了stm32的dsp库
3.更新了内存管理，现在可以使用task_insert_CCM函数将任务添加至stm32f4的cpu专属ccm内存中，理论上使用ccm内存的运行速度会快于普通内存，需要注意的是ccm内存区只有cpu可以访问，因此使用dma功能的
任务不要添加到ccm内存中
4.更新了mak_pid.c，搬运了去年的云台pid部分内容。
2019.9.18
更新文件结构，需要用户修改的文件全部放在user_app和user_other中，基本分离了操作系统底层与用户文件。之后开发的时候也注意这一点。
2019.9.23
修改了task_remote的机制，用户可以直接在自己的文件中读取remote_mag
调用示例：
	#include "headfile.h"
	void	task_example(void* param)
	{
		p_remote_data	data;
		S16 temp,temp2;
		while(1)
		{
			data=msg_get_read_some(&remote_msg);	//注意这里的读写消息函数必须是_some
			if(data!=NULL)
			{
				temp=((S16)data->LL)-1024;			//LL是新加的遥控器左上方滚轮的值，减去1024后是+-660
				temp2=data->KEY_data.Key_W;			//键盘上W是否被按下，按下为1，未按下是0
				msg_read_finish_some(&remote_msg);
			}
			task_delay_ms(5);
		}
	}
	
2019.10.4
更新了task_ANO即新匿名上位机程序，接口改为全局变量，使用方便。使用说明见ANO.txt
2019.10.5

device中加入rng_led.c，即硬件随机数功能，接口详见rng_led.h（已加入headfile.h中）
user_other中加入mak_filter.c，提供一些常用滤波函数，目前移植了两款网上的二阶卡尔曼滤波函数

2019.11.10
底层更新：
修复了oled交互界面在show界面按确认键程序卡死的bug

使用headfile.h中的宏定义区分主控板类型

陀螺仪用户接口更改为全局变量：
#include "task_mpu.h"
IMU_data.gyro_z (float 角速度逆时针为正 +-32767对应+-1000deg/s 1000hz更新)
IMU_data.yaw（float 偏航角逆时针为正 +-180 200hz更新）

2020.10.30
v1.0
目前完成21赛季哨兵初测代码
已经更新：摩擦轮量程：1100-1990
	遥控器消息机制：统一了接受数据结构，遥控器响应足够
	UI控制snail摩擦轮：UI后期还需更新！
2020.11.3
v1.1
3508双环控制不带载测试通过
6020双环测试通过不带载测试通过

需要更新：//print_wave有点问题，需要根据步兵的修改（串口通讯）
	3508双环控制，预计加电流环
	6020双环控制，预计加位置环
	超声波
	二代哨兵上车调运行状态（通过遥控器：底盘、云台、发弹）
自动方面：
	二代哨兵底盘运动决策（视觉+超声波+功率控制）
	二代哨兵发弹逻辑与热量控制（视觉）
	二代哨兵云台运动决策（视觉+发弹）
2020-12-4 
v1.2
上下型哨兵基础功能测试完成，目前存在问题：
云台自动、手动切换时存在积分饱和问题，尝试使用抗积分pid试一试。
底盘超声波还未尝试，底盘巡航运动未测
视觉联调一次，现需要在32上读取帧率，上云台测效果
发弹逻辑、运动逻辑未确定