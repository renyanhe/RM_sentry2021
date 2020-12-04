#ifndef _AMPLITUDE_LIMITING_AND_LPF_H_
#define _AMPLITUDE_LIMITING_AND_LPF_H_
#include "headfile.h"

float amplitude_limiting(float current_data,float old_data,float limit);  //限幅通用函数
float LPF(float right,float current_data,float old_data);//低通滤波器，参数为分配给当前值的权重

#endif