#ifndef _AMPLITUDE_LIMITING_AND_LPF_H_
#define _AMPLITUDE_LIMITING_AND_LPF_H_
#include "headfile.h"

float amplitude_limiting(float current_data,float old_data,float limit);  //限幅通用函数
float LPF(float right,float current_data,float old_data);//低通滤波器，参数为分配给当前值的权重

typedef struct
{
float Frep;
float Amp;
float rise;
} sin_test_t;//位置环测试时，正弦幅值0.3,频率6Hz。速度测试时，正弦幅值0.5额定速度,频率10Hz。电流环测试时,正弦幅值0.4额定值,正弦幅值20Hz。

extern sin_test_t Pos_sintest;
extern sin_test_t speed_sintest;
extern sin_test_t current_sintest;
void sin_test_init(sin_test_t *SIN, float fre, float amp, float rise);
float sin_test(sin_test_t *SIN);//单位：Hz、度

#endif

