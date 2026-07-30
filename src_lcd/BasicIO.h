#ifndef _BASICIO_H
#define _BASICIO_H



// GPIO Input
#define IO_KEY					GPIO_NUM_14

// GPIO Output
#define IO_LED					GPIO_NUM_21
#define IO_DISP					GPIO_NUM_20

// ADC Input
#define ADC_UNIT				ADC_UNIT_1 // unit 1 (can be used with RF)
#define ADC_CHANNEL_NTC			ADC_CHANNEL_0 // Channel0 <-> GPIO1
#define ADC_CHANNEL_LDR			ADC_CHANNEL_1 // Channel1 <-> GPIO2
#define ADC_RESOLUTION			ADC_BITWIDTH_DEFAULT // defalut 12bits (0-4095)
#define ADC_ATTEN				ADC_ATTEN_DB_12 // 12dB attenuation (0-3.3V)

// PWM Output
// Backlight
#define IO_LIGHT				GPIO_NUM_4
#define PWM_TIMER_LIGHT			LEDC_TIMER_0
#define PWM_CHANNEL_LIGHT		LEDC_CHANNEL_0
#define PWM_FREQ_LIGHT			(30000) // 30kHz
#define PWM_RESOLUTION_LIGHT	LEDC_TIMER_11_BIT // 最大 11 位 (0~2047)
// Fan
#define IO_FAN					GPIO_NUM_45
#define PWM_TIMER_FAN			LEDC_TIMER_1
#define PWM_CHANNEL_FAN			LEDC_CHANNEL_1
#define PWM_FREQ_FAN			(10000) // 10kHz
#define PWM_RESOLUTION_FAN		LEDC_TIMER_12_BIT // 最大 12 位 (0~4095)





void io_main(void);



#endif



