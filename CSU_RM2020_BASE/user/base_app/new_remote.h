#ifndef _NEWR_H_
#define _NEWR_H_
#include "headfile.h"

#define SHOOT (0101U)
#define SHOOT_2 (0102U)
#define UNSHOOT (0103U)

typedef struct _test_remote_                  //定义的哨兵结构体
{
	 int yaw;
   int pitch;
	 int yaw_2;
   int pitch_2;
	 U8	SL_pull;//2006拨弹
	 int speed;//底盘速度
	 u8 SW_right;//右拨杆
	 U8 SW_left;//左拨杆
}sentry_t,*p_sentry;
extern msg_t sentry_msg;



#endif
