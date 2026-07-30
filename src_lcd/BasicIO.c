#include "BasicIO.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"


static adc_oneshot_unit_handle_t adc1_handle = NULL;


void init_gpio(void) // GPIO input output init
{
	// Input KEY
	gpio_config_t io_conf_in = {
		.pin_bit_mask = (1ULL << IO_KEY),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE, // pull up
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE, // no interrupt
	};
	gpio_config(&io_conf_in);

	// Output LED
	gpio_config_t io_conf_out = {
		.pin_bit_mask = (1ULL << IO_LED),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&io_conf_out);

	// Output DISP
	gpio_config_t io_conf_out_disp = {
		.pin_bit_mask = (1ULL << IO_DISP),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&io_conf_out_disp);
}

void init_adc(void) // ADC init
{
	// create ADC unit
	adc_oneshot_unit_init_cfg_t init_config1 = {
		.unit_id = ADC_UNIT,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

	// ADC channel config
	adc_oneshot_chan_cfg_t config = {
		.bitwidth = ADC_RESOLUTION, // resolution
		.atten = ADC_ATTEN, // attenuation
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_NTC, &config));
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_LDR, &config));
}

void init_pwm(void) // PWM LEDC init
{
	// Backlight PWM
	ledc_timer_config_t timer_light = {
		.speed_mode       = LEDC_LOW_SPEED_MODE,
		.timer_num        = PWM_TIMER_LIGHT,
		.freq_hz          = PWM_FREQ_LIGHT,
		.duty_resolution  = PWM_RESOLUTION_LIGHT,
		.clk_cfg          = LEDC_AUTO_CLK
	};
	ledc_timer_config(&timer_light);

	ledc_channel_config_t channel_20k = {
		.speed_mode     = LEDC_LOW_SPEED_MODE,
		.channel        = PWM_CHANNEL_LIGHT,
		.timer_sel      = PWM_TIMER_LIGHT,
		.gpio_num       = IO_LIGHT,
		.duty           = 0, // initial duty cycle
		.hpoint         = 0
	};
	ledc_channel_config(&channel_20k);

	// Cooling fan PWM
	ledc_timer_config_t timer_fan = {
		.speed_mode       = LEDC_LOW_SPEED_MODE,
		.timer_num        = PWM_TIMER_FAN,
		.freq_hz          = PWM_FREQ_FAN,
		.duty_resolution  = PWM_RESOLUTION_FAN,
		.clk_cfg          = LEDC_AUTO_CLK
	};
	ledc_timer_config(&timer_fan);

	ledc_channel_config_t channel_10k = {
		.speed_mode     = LEDC_LOW_SPEED_MODE,
		.channel        = PWM_CHANNEL_FAN,
		.timer_sel      = PWM_TIMER_FAN,
		.gpio_num       = IO_FAN,
		.duty           = 0, // initial duty cycle
		.hpoint         = 0
	};
	ledc_channel_config(&channel_10k);
}

void io_main(void)
{
	init_gpio();
	init_pwm();
	init_adc();

	int led_state = 0;
	int pwm_duty = 0;

	while (0) {
		// read key (invert logic)
		int btn_val = gpio_get_level(IO_KEY);
		
		// LED blink (GPIO toggle)
		led_state = !led_state;
		gpio_set_level(IO_LED, led_state);

		// read ADC
		int adc_ntc = 0, adc_ldr = 0;
		ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_NTC, &adc_ntc));
		ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_LDR, &adc_ldr));

		// PWM
		pwm_duty += 100;
		if (pwm_duty >= 2048) {
			pwm_duty = 0;
		}
		ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_LIGHT, pwm_duty));
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_LIGHT));
		ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_FAN, 4095));
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_FAN));

		printf("KEY: %d | LED: %d | ADC: %d %d | PWM: %d\n", 
				btn_val, led_state, adc_ntc, adc_ldr, pwm_duty);

		vTaskDelay(pdMS_TO_TICKS(200));
	}
	gpio_set_level(IO_DISP, 1);
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_LIGHT, 500));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_LIGHT));
}







