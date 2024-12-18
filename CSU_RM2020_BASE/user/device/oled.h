//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//中景园电子
//店铺地址：http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  文 件 名   : main.c
//  版 本 号   : v2.0
//  作    者   : HuangKai
//  生成日期   : 2014-0101
//  最近修改   : 
//  功能描述   : OLED 4接口演示例程(51系列)
//              说明: 
//              ----------------------------------------------------------------
//              GND    电源地
//              VCC  接5V或3.3v电源
//              D0   接PD6（SCL）
//              D1   接PD7（SDA）
//              RES  接PD4
//              DC   接PD5
//              CS   接PD3               
//              ----------------------------------------------------------------
// 修改历史   :
// 日    期   : 
// 作    者   : HuangKai
// 修改内容   : 创建文件
//版权所有，盗版必究。
//Copyright(C) 中景园电子2014/3/16
//All rights reserved
//******************************************************************************/
#ifndef __OLED_H
#define __OLED_H			  	 
#include "stdlib.h"	  
#include "headfile.h"

#define OLED_ADDRESS 0x78
#define SIZE 8
#define Max_Column	128
    						  
//-----------------OLED端口定义----------------

#define OLED_CS_Clr()  {}
#define OLED_CS_Set()  {}

#define OLED_SDIN_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_13)  //OLED4
#define OLED_SDIN_Set() GPIO_SetBits(GPIOD,GPIO_Pin_13)

#define OLED_RST_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_10) //OLED1
#define OLED_RST_Set() GPIO_SetBits(GPIOD,GPIO_Pin_10)

#define OLED_SCLK_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_12)//OLED3
#define OLED_SCLK_Set() GPIO_SetBits(GPIOD,GPIO_Pin_12)

#define OLED_DC_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_11)//OLED2
#define OLED_DC_Set() GPIO_SetBits(GPIOD,GPIO_Pin_11)	
 		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据


//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd,u8 type);   
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 type);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size,u8 type);
void OLED_ShowString(u8 x,u8 y, u8 *p,u8 type);
void OLED_PrintVar(uint8_t x, uint8_t y, double data, uint8_t Length, uint8_t num);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x,u8 y,u8 no,u8 typr);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[],u8 type);
void OLED_PrintAD(uint8_t x, uint8_t y, int16_t data, int16_t DataMax);
signed int OLED_printf(uint8_t x,uint8_t y,const char *pFormat, ...);
#endif  

