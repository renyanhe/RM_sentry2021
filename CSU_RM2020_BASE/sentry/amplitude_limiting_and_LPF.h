#ifndef _AMPLITUDE_LIMITING_AND_LPF_H_
#define _AMPLITUDE_LIMITING_AND_LPF_H_
#include "headfile.h"

float amplitude_limiting(float current_data,float old_data,float limit);  //�޷�ͨ�ú���
float LPF(float right,float current_data,float old_data);//��ͨ�˲���������Ϊ�������ǰֵ��Ȩ��

#endif