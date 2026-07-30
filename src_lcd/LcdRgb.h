#ifndef _LCDRGB_H
#define _LCDRGB_H

#include "BasicIO.h"

#include "driver/ledc.h"

// 屏幕尺寸
#define LCD_H_RES				800
#define LCD_V_RES				480
#define LCD_FRAME_SIZE			(LCD_H_RES * LCD_V_RES * 1)

// RGB 流控引脚
#define LCD_PIN_PCLK			GPIO_NUM_19
#define LCD_PIN_DE				GPIO_NUM_13
#define LCD_PIN_VSYNC			GPIO_NUM_46
#define LCD_PIN_HSYNC			GPIO_NUM_3

// RGB 数据引脚 (8位或16位)
#define LCD_PIN_DATA0			GPIO_NUM_18
#define LCD_PIN_DATA1			GPIO_NUM_8
#define LCD_PIN_DATA2			GPIO_NUM_15
#define LCD_PIN_DATA3			GPIO_NUM_16
#define LCD_PIN_DATA4			GPIO_NUM_17
#define LCD_PIN_DATA5			GPIO_NUM_5
#define LCD_PIN_DATA6			GPIO_NUM_6
#define LCD_PIN_DATA7			GPIO_NUM_7

// 色彩定义
#define COLOR_K			0x00 // 黑
#define COLOR_A			0x6E // 灰
#define COLOR_W			0xFF // 白
#define COLOR_R			0xE0 // 红
#define COLOR_G			0x1C // 绿
#define COLOR_B			0x03 // 蓝
#define COLOR_Y			0xF8 // 黄
#define COLOR_C			0x1F // 青
#define COLOR_M			0xE3 // 品


void init_rgb(void);
void rgb_test(void);


#endif



