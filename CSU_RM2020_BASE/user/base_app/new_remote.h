#ifndef _NEWR_H_
#define _NEWR_H_
#include "headfile.h"

#define SHOOT (0101U)
#define SHOOT_2 (0102U)
#define UNSHOOT (0103U)

typedef struct _test_remote_                  //������ڱ��ṹ��
{
	 int yaw;
   int pitch;
	 int yaw_2;
   int pitch_2;
	 U8	SL_pull;//2006����
	 int speed;//�����ٶ�
	 u8 SW_right;//�Ҳ���
	 U8 SW_left;//�󲦸�
}sentry_t,*p_sentry;
extern msg_t sentry_msg;



#endif
