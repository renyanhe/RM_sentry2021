#ifndef _USER_USART_H
#define _USER_USART_H

#include "makos_includes.h"
extern volatile float yaw_new2020,pitch_new2020,dist_new2020,yaw_angle2021,pitch_angle2021;
void	usart1_init(void);
void    uasrt6_init(u32 baud);
extern volatile float __count;
void usart2_send_char(char ch);
void usart2_send_string(u8 *buff, u32 len);
void usart3_send_char(char ch);
void usart3_send_string(u8 *buff, u32 len);
void usart6_send_char(char ch);
void usart6_send_string(u8 *buff, u32 len);

void print_wave(u8 number, u8 length, ...);

#endif
