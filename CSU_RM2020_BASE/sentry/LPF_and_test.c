#include "LPF_and_test.h"

float amplitude_limiting(float current_data,float old_data,float limit)  //�޷�ͨ�ú���
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

float LPF(float right,float current_data,float old_data)//��ͨ�˲���������Ϊ�������ǰֵ��Ȩ��
{
   float final_data;
	 final_data = right*current_data + (1-right)*old_data ;
	 return final_data;
}

void sin_test_init(sin_test_t *SIN, float fre, float amp, float rise)
{
	 SIN->Frep = fre;
	 SIN->Amp = amp;
	 SIN->rise = rise;
}

float sin_test(sin_test_t *SIN)
{
	 float y,x = 1;
	 y = SIN->Amp*sin(SIN->Frep*6.283*(x*3.1415/180.0));
	 x += SIN->rise;
}
