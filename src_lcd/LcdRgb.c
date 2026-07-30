#include "LcdRgb.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_log.h"

esp_lcd_panel_handle_t panel_handle = NULL; // Device handle for LCD panel

uint8_t *LCD_Buf = NULL, *LCD_Buf0 = NULL, *LCD_Buf1 = NULL; // Selected buffer pointer and frame buffer pointers

void init_rgb(void)
{
	// 1. Configure RGB panel
	esp_lcd_rgb_panel_config_t rgb_cfg = {
		.clk_src = LCD_CLK_SRC_DEFAULT,
		.timings = {
			// Clock frequency
			.pclk_hz = 20 * 1000 * 1000, // 20MHz
			// Display resolution
			.h_res = LCD_H_RES,
			.v_res = LCD_V_RES,
			// Timing parameters
			.hsync_pulse_width = 8,
			.hsync_back_porch = 10,
			.hsync_front_porch = 50,
			.vsync_pulse_width = 4,
			.vsync_back_porch = 16,
			.vsync_front_porch = 60,
		},
		.data_width = 8, // Number of data lines
		.de_gpio_num = LCD_PIN_DE,
		.pclk_gpio_num = LCD_PIN_PCLK,
		.vsync_gpio_num = LCD_PIN_VSYNC,
		.hsync_gpio_num = LCD_PIN_HSYNC,
		.data_gpio_nums = {
			LCD_PIN_DATA0,
			LCD_PIN_DATA1,
			LCD_PIN_DATA2,
			LCD_PIN_DATA3,
			LCD_PIN_DATA4,
			LCD_PIN_DATA5,
			LCD_PIN_DATA6,
			LCD_PIN_DATA7,
		},
		.flags.fb_in_psram = 1,		  // Allocate frame buffer in PSRAM
		.num_fbs = 2,				  // Allocate two frame buffers
		.flags.double_fb = 0,		  // Manual buffer switching
		.flags.refresh_on_demand = 0, // Auto refresh mode
	};

	// 2. Install RGB panel driver
	ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&rgb_cfg, &panel_handle));

	// 3. Initialize panel
	ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
	ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

	// Get addresses of automatically allocated frame buffers
	ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, (void **)&LCD_Buf0, (void **)&LCD_Buf1));
	printf("LCD_Buf0: 0x%lX, LCD_Buf1: 0x%lX\n", (uint32_t)LCD_Buf0, (uint32_t)LCD_Buf1);
}

// Tile configuration structure
typedef struct
{
	uint8_t *canvas;   // Canvas buffer pointer
	uint16_t canvas_w; // Canvas width
	uint16_t canvas_h; // Canvas height

	const uint8_t *matrix; // Color matrix pointer
	uint16_t matrix_m;	   // Color matrix rows (height)
	uint16_t matrix_n;	   // Color matrix columns (width)
	uint16_t block_w;	   // Width of each color block
	uint16_t block_h;	   // Height of each color block

	uint16_t start_x; // Fill area start X
	uint16_t start_y; // Fill area start Y
	uint16_t fill_w;  // Fill area total width
	uint16_t fill_h;  // Fill area total height
} TileConfig_t;

/**
 * @brief Perform tiled pattern filling on canvas
 * @param cfg Pointer to configuration structure
 */
void fill_tiled_pattern(const TileConfig_t *cfg)
{
	// Total pixel dimensions of base matrix unit
	uint16_t unit_pixel_w = cfg->matrix_n * cfg->block_w;
	uint16_t unit_pixel_h = cfg->matrix_m * cfg->block_h;

	// Traverse every pixel in fill area
	for (uint16_t ry = 0; ry < cfg->fill_h; ry++)
	{
		// Boundary check for Y axis
		if (cfg->start_y + ry >= cfg->canvas_h)
			break;

		for (uint16_t rx = 0; rx < cfg->fill_w; rx++)
		{
			// Boundary check for X axis
			if (cfg->start_x + rx >= cfg->canvas_w)
				break;

			// 1. Calculate pixel offset within base unit
			uint16_t inner_x = rx % unit_pixel_w;
			uint16_t inner_y = ry % unit_pixel_h;

			// 2. Calculate row and column index in color matrix
			uint16_t col_idx = inner_x / cfg->block_w;
			uint16_t row_idx = inner_y / cfg->block_h;

			// 3. Get color value
			uint16_t color = cfg->matrix[row_idx * cfg->matrix_n + col_idx];

			// 4. Write pixel to canvas
			uint32_t target_idx = (uint32_t)(cfg->start_y + ry) * cfg->canvas_w + (cfg->start_x + rx);
			cfg->canvas[target_idx] = color;
		}
	}
}

void fill_oblique_pattern(uint8_t *canvas, uint16_t canvas_w, uint16_t canvas_h)
{
	static uint16_t frameCnt = 0;
	frameCnt += 8;
	for (uint16_t y = 0; y < canvas_h; y++)
	{
		for (uint16_t x = 0; x < canvas_w; x++)
		{
			uint8_t r8 = frameCnt + y + x; // R component
			uint8_t g8 = frameCnt - y;	   // G component
			uint8_t b8 = frameCnt + y;	   // B component
			canvas[y * canvas_w + x] = (r8 & 0xE0) | ((g8 & 0xE0) >> 3) | ((b8 & 0xC0) >> 6);
		}
	}
}

const uint16_t fill_size_list[] = {
	500,
	400,
	400,
	400,
	400,
	300,
	400,
	200,
	300,
	200,
};

const uint8_t my_pattern[] = {
	COLOR_B,
	COLOR_G,
	COLOR_R,
	COLOR_W,
	COLOR_M,
	COLOR_Y,
	COLOR_C,
	COLOR_A,
};

void rgb_test(void)
{
	TileConfig_t cfg;
	cfg.canvas_w = LCD_H_RES;
	cfg.canvas_h = LCD_V_RES;
	cfg.matrix = (const uint8_t *)my_pattern;
	cfg.matrix_m = 2;
	cfg.matrix_n = 4;
	cfg.block_w = 100;
	cfg.block_h = 100;

	uint8_t pattern = 0;
	uint8_t key_old = 0;

	while (1)
	{
		ESP_LOGI("RGB", "test");

		if (key_old != !gpio_get_level(IO_KEY))
		{ // Edge detection
			key_old = !key_old;
			if (key_old)
			{ // Falling edge / pressed
				pattern = (pattern + 1) % 6;
				memset(LCD_Buf0, 0, LCD_FRAME_SIZE);
				memset(LCD_Buf1, 0, LCD_FRAME_SIZE);
			}
		}

		LCD_Buf = (LCD_Buf != LCD_Buf0) ? LCD_Buf0 : LCD_Buf1; // Double buffer toggle

		if (pattern < sizeof(fill_size_list) / (2 * sizeof(fill_size_list[0])))
		{
			cfg.canvas = LCD_Buf;
			cfg.fill_w = fill_size_list[pattern * 2 + 0];
			cfg.fill_h = fill_size_list[pattern * 2 + 1];
			cfg.start_x = (cfg.canvas_w - cfg.fill_w) / 2;
			cfg.start_y = (cfg.canvas_h - cfg.fill_h) / 2;
			fill_tiled_pattern(&cfg);
		}
		else
		{
			fill_oblique_pattern(LCD_Buf, LCD_H_RES, LCD_V_RES);
		}

		esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, LCD_Buf);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}