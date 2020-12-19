#include <unistd.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"  // xTaskCreate()

#include <esp_log.h>        // ESP_LOGI()
#include <nvs_flash.h>      // nvs_flash_init()
#include <esp_system.h>     // esp_restart()
#include <driver/gpio.h>    // gpio_set_level()

#include "wifi.h"           // initialise_wifi()
#include "ota_update.h"     // ota_perform_update()

static const char *TAG = "app";

#define GPIO_OUTPUT_ESP12_LED    2

static void gpio_init() {
	gpio_config_t io_conf;
	memset(&io_conf, 0, sizeof(io_conf));
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = 1ULL << GPIO_OUTPUT_ESP12_LED;
	io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&io_conf);
}


static void ota_update_task(void *pvParameter)
{
	while (true) {

		for (int cnt=60-1; cnt >=0; cnt--) {
			sleep(1);
			if (cnt%10 == 0) ESP_LOGI(TAG, "%d seconds remaining.", cnt);
		}

		for (int cnt=0 ; cnt < 10; cnt++) {    // 1 second in total
			gpio_set_level(GPIO_OUTPUT_ESP12_LED, 0);
			vTaskDelay(100 / portTICK_RATE_MS);    // 50ms
			gpio_set_level(GPIO_OUTPUT_ESP12_LED, 1);
			vTaskDelay(100 / portTICK_RATE_MS);    // 50ms
		}
		gpio_set_level(GPIO_OUTPUT_ESP12_LED, 0);
		if (ota_perform_update()) {
			ESP_LOGI(TAG, "Prepare to restart system!");
			esp_restart();
		}
		gpio_set_level(GPIO_OUTPUT_ESP12_LED, 1);
	}
}


void app_main()
{
	// Initialize NVS.
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
		// OTA app partition table has a smaller NVS partition size than the
		// non-OTA partition table. This size mismatch may cause NVS
		// initialization to fail.If this happens, we erase NVS partition and
		// initialize NVS again.
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK( err );

	gpio_init();

	initialise_wifi();
	wait_for_wifi();

	xTaskCreate(&ota_update_task, "ota_update_task", 8192, NULL, 5, NULL);
}
