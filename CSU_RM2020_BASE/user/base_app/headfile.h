#ifndef	_headfile_h
#define	_headfile_h

//����Ӳ������io�ڲ�һ����ѡ��������ͣ�
//MAIN_CONTROL_2019���������2019���ذ�
//MAIN_CONTROL_2020solider����ΰ��2020��������
//MAIN_CONTROL_2020sentry����������2020�ڱ�����
/***********��ѡһ��������*******/

//#define MAIN_CONTROL_2019
//#define MAIN_CONTROL_2020solider
#define MAIN_CONTROL_2020sentry

/*********************************/

#if defined(MAIN_CONTROL_2020solider)||defined(MAIN_CONTROL_2020sentry)
#define USE_NEW_OLED
#endif

//****MAKOS_RTOS****
#include "makos_includes.h"
//****other****
#include "data_fifo.h"
#include "protocol.h"

//****arithmetic****
#include "math.h"
#include "arm_math.h"

//****device****
#include "flash_data.h"
#include "task_remote.h"
#include "timer.h"
#include "rng_led.h"

//****diver****
#include "led.h"
#include "stm32_flash.h"
#include "usart.h"
#include "user_usart.h"
#include "can.h"
#include "user_can.h"
#include "uart_dma.h"
#include "task_Ano.h"
#include "PWM.h"
#include "mak_filter.h"
#include "HC_ultrasonic.h"

//****ST_lib****
#include "string.h"
#include "stdlib.h"

//******sentry*******
#include "new_remote.h"
//#include "testbeach.h"
#include "pid.h"
#include "sentry_holder.h"
#include "sentry_classis.h"
#include "sentry_pull.h"
#include "sentry_ultrasonic.h"
#include "amplitude_limiting_and_LPF.h"

#include "communicate.h"
#include "judgement_info.h"
//#include "sentry_vision.h"

#endif
