/*
 * LCD12864.h
 *
 *  Created on: Feb 24, 2024
 *      Author: wolf.long
 */

#ifndef LCD12864_LCD12864_H_
#define LCD12864_LCD12864_H_

#include "main.h"
//IO写操作函数
#define lcd_sclk(x)    		HAL_GPIO_WritePin(GPIOB, TV_SCLK_Pin, x) 		//接口定义:lcd_sclk 就是LCD 的SCLK
#define lcd_sid(x)     		HAL_GPIO_WritePin(GPIOB, TV_SDA_Pin, x) 		//接口定义:lcd_sid 就是LCD 的SDA
#define lcd_rs(x)   	    HAL_GPIO_WritePin(GPIOB, TV_RS_Pin,x) 			//接口定义:lcd_rs 就是LCD 的RS,也叫“CD”
#define lcd_reset(x)   	    HAL_GPIO_WritePin(GPIOB, TV_RESET_Pin,x) 		//接口定义:lcd_reset 就是LCD 的RESET
#define lcd_cs1(x)   	    HAL_GPIO_WritePin(GPIOB, TV_CS_Pin,x) 			//接口定义:lcd_cs1 就是LCD 的CS1

#define Rom_IN(x)   	    HAL_GPIO_WritePin(GPIOA, ROM_IN_Pin,x) 			//字库IC 接口定义:Rom_IN 就是字库IC 的SI
#define Rom_OUT(x)   	    HAL_GPIO_WritePin(GPIOB, ROM_OUT_Pin,x) 		//字库IC 接口定义:Rom_OUT 就是字库IC 的SO
#define Rom_SCK(x)   	    HAL_GPIO_WritePin(GPIOB, ROM_SCK_Pin,x) 		//字库IC 接口定义:Rom_SCK 就是字库IC 的SCK
#define Rom_CS(x)   	    HAL_GPIO_WritePin(GPIOB, ROM_CS_Pin,x) 			//字库IC 接口定义Rom_CS 就是字库IC 的CS#

#define Rom_OUT_READ   	    HAL_GPIO_ReadPin(GPIOB, ROM_OUT_Pin) 		//输入SDA

#define uchar unsigned char
#define uint unsigned int
#define ulong unsigned long

void LCD12864_Rom_OUT();
void LCD12864_Rom_IN();
//写指令到LCD 模块
void transfer_command_lcd(int data1);
//写数据到LCD 模块
void transfer_data_lcd(int data1);
//LCD 模块初始化
void initial_lcd();
void lcd_address(uint page,uint column);
//全屏清屏
void clear_screen();
//显示128x64 点阵图像
void display_128x64(uchar *dp);
//显示16x16 点阵图像、汉字、生僻字或16x16 点阵的其他图标
void display_graphic_16x16(uchar page,uchar column,uchar *dp);
//显示8x16 点阵图像、ASCII, 或8x16 点阵的自造字符、其他图标
void display_graphic_8x16(uchar page,uchar column,uchar *dp);
//显示5X8 点阵图像、ASCII, 或5x8 点阵的自造字符、其他图标
void display_graphic_5x8(uchar page,uchar column,uchar *dp);
//送指令到晶联讯字库IC
void send_command_to_ROM( uchar datu);
//从指定地址读出数据写到液晶屏指定（page,column)座标中
void get_and_write_16x16(ulong fontaddr,uchar page,uchar column);
//从指定地址读出数据写到液晶屏指定（page,column)座标中
void get_and_write_8x16(ulong fontaddr,uchar page,uchar column);
//从指定地址读出数据写到液晶屏指定（page,column)座标中
void get_and_write_5x8(ulong fontaddr,uchar page,uchar column);
void display_GB2312_string(uchar page,uchar column,uchar *text);
void display_string_5x8(uchar page,uchar column,uchar *text);


extern uchar  bmp1[];
extern uchar  jiong1[];
extern uchar  lei1[];

#endif /* LCD12864_LCD12864_H_ */
