#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>

#include "esp_flash.h"
#include "esp_partition.h"

#include "esp_psram.h"
#include "esp_heap_caps.h"

#include "esp_efuse.h"
#include "esp_efuse_table.h"

#include "esp_chip_info.h"
#include "esp_private/esp_clk.h"

#include "LcdRgb.h"
#include "BasicIO.h"


void sys_info(void){
	printf("ESP-IDF Version: %s\n", esp_get_idf_version()); // 目前IDF版本5.5.3
	printf("FreeRTOS Tick Freq: %d Hz\n", configTICK_RATE_HZ); // FreeRTOS调度周期

	// 获取芯片基本信息
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	printf("Number of cores: %d\n", chip_info.cores);
	printf("CPU Freq: %d MHz\n", esp_clk_cpu_freq() / 1000000);

	// 获取 Flash 容量
	uint32_t flash_size;
	if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
		printf("Flash Size: %lu MB\n", flash_size / (1024 * 1024));
	}

	// 获取 PSRAM 容量 (若未配置或没有则返回0)
	if (esp_psram_is_initialized()) {
		printf("PSRAM Status: ON, Size: %d MB\n", esp_psram_get_size() / (1024 * 1024));
	} else {
		printf("PSRAM Status: OFF\n");
	}

	// --- 动态内存统计 ---
	// 内部内存 (SRAM)
	size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
	size_t min_internal  = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
	printf("Remaining SRAM: %d B (Historical Min: %d B)\n", free_internal, min_internal);

	// 外部内存 (PSRAM)
	if (esp_psram_is_initialized()) {
		size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
		printf("Remaining PSRAM: %d B\n", free_psram);
	}
}

void efuse_read(void) {
	uint32_t value = 0;

	esp_efuse_read_field_blob(ESP_EFUSE_VDD_SPI_FORCE, &value, 1);
	printf("VDD_SPI_FORCE: %lu\n", value);

	esp_efuse_read_field_blob(ESP_EFUSE_VDD_SPI_TIEH, &value, 1);
	printf("VDD_SPI_TIEH: %lu\n", value);
}

void check_memory() {
	// 检查总的外置内存
	size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
	size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
	
	printf("PSRAM total size: %d bytes (%d MB)\n", psram_size, psram_size / 1024 / 1024);
	printf("PSRAM free size: %d bytes\n", free_psram);

	// 测试申请一段大的内存
	void* test_ptr = heap_caps_malloc(1024 * 1024 * 2, MALLOC_CAP_SPIRAM); // 申请 2MB
	if (test_ptr != NULL) {
		ESP_LOGI("MEM", "PSRAM malloc succeed.");
		heap_caps_free(test_ptr);
	} else {
		ESP_LOGE("MEM", "PSRAM malloc failed.");
	}
}

void check_flash() {
	uint32_t flash_size;
	esp_flash_get_size(NULL, &flash_size);
	printf("Flash size: %lu MB\n", flash_size / (1024 * 1024));
	
	// 打印分区表信息
	esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
	while (it != NULL) {
		const esp_partition_t *p = esp_partition_get(it);
		printf("partition: %s, size: %ld KB\n", p->label, p->size / 1024);
		it = esp_partition_next(it);
	}
}

static const char *TAG = "app_main"; // 定义日志标签

void app_main(void)
{
	io_main();
	init_rgb();
	rgb_test();
	while(1){
		ESP_LOGI(TAG, "Hello ESP32S3!"); // 输出日志到串口
		vTaskDelay(pdMS_TO_TICKS(1000)); // 延时1000ms
	}
}
