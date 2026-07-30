#ifndef LCD_H
#define LCD_H

#include "driver/ledc.h"
#include "driver/gpio.h"

// Screen resolution
#define LCD_H_RES               800
#define LCD_V_RES               480
#define LCD_FRAME_SIZE          (LCD_H_RES * LCD_V_RES * 1)

// RGB control pins
#define LCD_PIN_PCLK            GPIO_NUM_19
#define LCD_PIN_DE              GPIO_NUM_13
#define LCD_PIN_VSYNC           GPIO_NUM_46
#define LCD_PIN_HSYNC           GPIO_NUM_3

// RGB data pins
#define LCD_PIN_DATA0           GPIO_NUM_18
#define LCD_PIN_DATA1           GPIO_NUM_8
#define LCD_PIN_DATA2           GPIO_NUM_15
#define LCD_PIN_DATA3           GPIO_NUM_16
#define LCD_PIN_DATA4           GPIO_NUM_17
#define LCD_PIN_DATA5           GPIO_NUM_5
#define LCD_PIN_DATA6           GPIO_NUM_6
#define LCD_PIN_DATA7           GPIO_NUM_7

// Display enable & backlight pins
#define IO_DISP                 GPIO_NUM_20
#define IO_LIGHT                GPIO_NUM_4
#define IO_KEY                  GPIO_NUM_14

#define PWM_TIMER_LIGHT         LEDC_TIMER_0
#define PWM_CHANNEL_LIGHT       LEDC_CHANNEL_0
#define PWM_FREQ_LIGHT          (30000)
#define PWM_RESOLUTION_LIGHT    LEDC_TIMER_11_BIT

// Colors
#define COLOR_K                 0x00
#define COLOR_A                 0x6E
#define COLOR_W                 0xFF
#define COLOR_R                 0xE0
#define COLOR_G                 0x1C
#define COLOR_B                 0x03
#define COLOR_Y                 0xF8
#define COLOR_C                 0x1F
#define COLOR_M                 0xE3

void init_rgb(void);
void rgb_test(void);

#endif /* LCD_H */
