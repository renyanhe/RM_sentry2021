#include "task_mpu.h"
#include "mpu.h"
#include "imu_packet.h"
#include "imu_data_decode.h"

imu_data_t IMU_data;//全局变量用户接口

float mpu_gyro_z_offest;	//陀螺仪零飘补偿
void gyro_offest(void)
{
	int i;
	static float last_offest_gyro;
	retry:
	for(i=0;i<300;i++)
	{
		READ_MPU6500_GYRO();
		if(fabs(last_offest_gyro-mpu_value.Gyro[2])<1500)
		{
			mpu_gyro_z_offest+=mpu_value.Gyro[2];
			last_offest_gyro=mpu_value.Gyro[2];
		}
		else
		{
			mpu_gyro_z_offest=0;
			last_offest_gyro=0;
			goto retry;
		}
	}
	mpu_gyro_z_offest/=300;
}


void	task_mpu(void* param)
{
	static float Eular[3];		//串口陀螺仪反馈的欧拉角
	
	Init_MPU6500();
	gyro_offest();
	while(1)
	{
		get_eular(Eular);
		READ_MPU6500_GYRO();
		mpu_value.Gyro[2]-=mpu_gyro_z_offest;
		
		IMU_data.gyro_z=mpu_value.Gyro[2];
		IMU_data.yaw=Eular[2];

		task_delay_ms(5);
	}
}
