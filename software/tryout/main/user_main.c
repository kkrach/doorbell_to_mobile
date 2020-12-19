/* gpio example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wps.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"


#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


static const char *TAG = "kkr";

#define WPS_TEST_MODE WPS_TYPE_PBC

/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO15: output
 * GPIO16: output
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Test:
 * Connect GPIO15 with GPIO4
 * Connect GPIO16 with GPIO5
 * Generate pulses on GPIO15/16, that triggers interrupt on GPIO4/5
 *
 */

#define GPIO_OUTPUT_ESP12_LED    2
#define GPIO_OUTPUT_OPEN        14
#define GPIO_OUTPUT_MUTE        13
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_ESP12_LED) | (1ULL<<GPIO_OUTPUT_OPEN) | (1ULL<<GPIO_OUTPUT_MUTE))
#define GPIO_INPUT_WPS_BUTTON          4
#define GPIO_INPUT_RING_HOUSE_DETECT   5
#define GPIO_INPUT_RING_HOME_DETECT   12
#define GPIO_INPUT_PIN_PULLDOWN  ((1ULL<<GPIO_INPUT_RING_HOUSE_DETECT) | (1ULL<<GPIO_INPUT_RING_HOME_DETECT))
#define GPIO_INPUT_PIN_PULLUP    ((1ULL<<GPIO_INPUT_WPS_BUTTON))

static xQueueHandle gpio_evt_queue = NULL;

#define TSK_DELAY_FAST  200  // 5x per second
#define TSK_DELAY_INIT  500  // 2x per second
#define TSK_DELAY_SLOW 2000  // 0.5x per second
static int tsk_delay = TSK_DELAY_INIT;

static esp_wps_config_t config = WPS_CONFIG_INIT_DEFAULT(WPS_TEST_MODE);


static int gpio_ring_house_cnt = 0;
static int gpio_ring_home_cnt = 0;
static int gpio_err_cnt = 0;


#define HOST_IPV4 1
#define HOST_IP_ADDR "192.168.178.100"
#define HOST_PORT   16983

static char payload[1024] = { '\0' };


static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

#ifdef HOST_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(HOST_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(HOST_PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        while (1) {

            sprintf(payload, "house=%d home=%d err=%d\n", gpio_ring_house_cnt, gpio_ring_home_cnt, gpio_err_cnt);

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            struct sockaddr_in sourceAddr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(sourceAddr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

            // Error occured during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                if (strncmp("MUTE", rx_buffer, 4) == 0) {
                    ESP_LOGI(TAG, "Muting RITTO...");
                    gpio_set_level(GPIO_OUTPUT_MUTE, 1);
                    vTaskDelay(1000 / portTICK_RATE_MS);
                    gpio_set_level(GPIO_OUTPUT_MUTE, 0);
                }
                if (strncmp("UNMUTE", rx_buffer, 6) == 0) {
                    ESP_LOGI(TAG, "Unmuting RITTO...");
                    gpio_set_level(GPIO_OUTPUT_MUTE, 1);
                    vTaskDelay(1000 / portTICK_RATE_MS);
                    gpio_set_level(GPIO_OUTPUT_MUTE, 0);
                }
                else if (strncmp("OPEN", rx_buffer, 4) == 0) {
                    ESP_LOGI(TAG, "Opening RITTO...");
                    gpio_set_level(GPIO_OUTPUT_OPEN, 1);
                    vTaskDelay(1000 / portTICK_RATE_MS);
                    ESP_LOGI(TAG, "Opening RITTO done.");
                    gpio_set_level(GPIO_OUTPUT_OPEN, 0);
                }
                else {
                    ESP_LOGI(TAG, "Cannot parse message '%s'...", rx_buffer);
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

static esp_err_t event_handler(void* ctx, system_event_t* event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
            break;

        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
            ESP_LOGI(TAG, "got ip:%s\n",
                     ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));

            xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
            tsk_delay = TSK_DELAY_SLOW;
            break;

        case SYSTEM_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;

        case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
            /*point: the function esp_wifi_wps_start() only get ssid & password
             * so call the function esp_wifi_connect() here
             * */
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_WPS_ER_SUCCESS");
            ESP_ERROR_CHECK(esp_wifi_wps_disable());
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;

        case SYSTEM_EVENT_STA_WPS_ER_FAILED:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_WPS_ER_FAILED");
            ESP_ERROR_CHECK(esp_wifi_wps_disable());
            ESP_ERROR_CHECK(esp_wifi_wps_enable(&config));
            ESP_ERROR_CHECK(esp_wifi_wps_start(0));
            break;

        case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_WPS_ER_TIMEOUT");
            ESP_ERROR_CHECK(esp_wifi_wps_disable());
            ESP_ERROR_CHECK(esp_wifi_wps_enable(&config));
            ESP_ERROR_CHECK(esp_wifi_wps_start(0));
            break;

        case SYSTEM_EVENT_STA_WPS_ER_PIN:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_WPS_ER_PIN");
            /*show the PIN code here*/
            ESP_LOGI(TAG, "WPS_PIN = NOT IMPLEMENTED!");
            break;

        default:
            break;
    }

    return ESP_OK;
}

static void start_wifi() {
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_FLASH) );


    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_err_t wifi_status = esp_wifi_connect();

    switch (wifi_status) {
    case ESP_OK:
        ESP_LOGI(TAG, "WIFI connect succeeded!");
        break;
    case ESP_ERR_WIFI_NOT_INIT:
        ESP_LOGI(TAG, "WiFi is not initialized by esp_wifi_init");
        break;
    case ESP_ERR_WIFI_NOT_STARTED:
        ESP_LOGI(TAG, "WiFi is not started by esp_wifi_start");
        break;
    case ESP_ERR_WIFI_CONN:
        ESP_LOGI(TAG, "WiFi internal error, station or soft-AP control block wrong");
        break;
    case ESP_ERR_WIFI_SSID:
        ESP_LOGI(TAG, "SSID of AP which station connects is invalid");
        break;
    default:
        ESP_LOGI(TAG, "Unknown WIFI status code: %d", wifi_status);
        break;
    }
}

/*init wifi as sta and start wps*/
static void start_wps(void)
{
    ESP_LOGI(TAG, "start wps...");

    ESP_ERROR_CHECK(esp_wifi_wps_enable(&config));
    ESP_ERROR_CHECK(esp_wifi_wps_start(0));
}
static void gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void led_task(void *pvParameters)
{
    int cnt = 0;

    while (1) {
        cnt++;
        vTaskDelay(tsk_delay / portTICK_RATE_MS);
        gpio_set_level(GPIO_OUTPUT_ESP12_LED, cnt % 2);
//        gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);
    }

    vTaskDelete(NULL);
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    gpio_config_t io_conf;

    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(GPIO_OUTPUT_MUTE, 0);
    gpio_set_level(GPIO_OUTPUT_OPEN, 0);

    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_PULLUP;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_PULLDOWN;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);


    //change gpio intrrupt type for one pin
//    gpio_set_intr_type(GPIO_INPUT_WPS_BUTTON, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));


    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_WPS_BUTTON, gpio_isr_handler, (void *) GPIO_INPUT_WPS_BUTTON);
    gpio_isr_handler_add(GPIO_INPUT_RING_HOUSE_DETECT, gpio_isr_handler, (void *) GPIO_INPUT_RING_HOUSE_DETECT);
    gpio_isr_handler_add(GPIO_INPUT_RING_HOME_DETECT, gpio_isr_handler, (void *) GPIO_INPUT_RING_HOME_DETECT);


    xTaskCreate(led_task, "led_task", 2048, NULL, 10, NULL);

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            switch (io_num) {
            case GPIO_INPUT_WPS_BUTTON:
                if (tsk_delay == TSK_DELAY_SLOW) {
                    tsk_delay = TSK_DELAY_FAST;
                    start_wps();
                }
                else {
                    tsk_delay = TSK_DELAY_SLOW;
                    ESP_ERROR_CHECK(esp_wifi_wps_disable());
                }
                break;
            case GPIO_INPUT_RING_HOUSE_DETECT:
                gpio_ring_house_cnt++;
                break;
            case GPIO_INPUT_RING_HOME_DETECT:
                gpio_ring_home_cnt++;
                break;

            default:
                gpio_err_cnt++;
                break;
            }
        }
    }
}





void app_main(void)
{
    /* Initialize NVS — it is used to store PHY calibration data */
    ESP_ERROR_CHECK(nvs_flash_init());

    start_wifi();

    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    while (1) {
        ESP_LOGI(TAG, "alive");
        vTaskDelay(10*1000 / portTICK_RATE_MS);
    }

}
