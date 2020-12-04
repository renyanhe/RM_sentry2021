#include "amplitude_limiting_and_LPF.h"

float amplitude_limiting(float current_data,float old_data,float limit)  //限幅通用函数
{
	 float final_data;
   if(current_data-old_data >=limit || current_data -old_data <=-limit)
   {
	    current_data = old_data ;
	 }
	 final_data =current_data;
	 old_data =current_data;
	 return final_data;
}

float LPF(float right,float current_data,float old_data)//低通滤波器，参数为分配给当前值的权重
{
   float final_data;
	 final_data = right*current_data + (1-right)*old_data ;
	 return final_data;
}