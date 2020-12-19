#ifndef _AMPLITUDE_LIMITING_AND_LPF_H_
#define _AMPLITUDE_LIMITING_AND_LPF_H_
#include "headfile.h"

float amplitude_limiting(float current_data,float old_data,float limit);  //�޷�ͨ�ú���
float LPF(float right,float current_data,float old_data);//��ͨ�˲���������Ϊ�������ǰֵ��Ȩ��

typedef struct
{
float Frep;
float Amp;
float rise;
} sin_test_t;//λ�û�����ʱ�����ҷ�ֵ0.3,Ƶ��6Hz���ٶȲ���ʱ�����ҷ�ֵ0.5��ٶ�,Ƶ��10Hz������������ʱ,���ҷ�ֵ0.4�ֵ,���ҷ�ֵ20Hz��

extern sin_test_t Pos_sintest;
extern sin_test_t speed_sintest;
extern sin_test_t current_sintest;
void sin_test_init(sin_test_t *SIN, float fre, float amp, float rise);
float sin_test(sin_test_t *SIN);//��λ��Hz����

#endif

