/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * W6100 Basic Example for ESP32-S3-DevKitC
 *
 * Pin Configuration:
 *   MOSI: GPIO11
 *   MISO: GPIO13
 *   SCLK: GPIO12
 *   CS:   GPIO10
 *   INT:  GPIO14
 *   RST:  GPIO21
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_http_client.h"
#include "w6100.h"

static const char *TAG = "w6100_example";

// Download test configuration
#define TEST_URL "http://speedtest.tele2.net/100MB.zip"
#define RECV_BUF_SIZE (64 * 1024)
#define PROGRESS_INTERVAL_SEC 5

// Event group to signal when we have an IP address
static EventGroupHandle_t s_eth_event_group;
#define ETH_GOT_IP_BIT BIT0

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "IP address: " IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "Netmask:    " IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "Gateway:    " IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");

    // Signal that we got an IP
    xEventGroupSetBits(s_eth_event_group, ETH_GOT_IP_BIT);
}

// Helper function to get CPU idle times
static void get_cpu_idle_times(configRUN_TIME_COUNTER_TYPE *total_run_time,
                                configRUN_TIME_COUNTER_TYPE *idle0_time,
                                configRUN_TIME_COUNTER_TYPE *idle1_time)
{
    *idle0_time = 0;
    *idle1_time = 0;
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    TaskStatus_t *task_array = malloc(sizeof(TaskStatus_t) * num_tasks);
    if (task_array) {
        num_tasks = uxTaskGetSystemState(task_array, num_tasks, total_run_time);
        for (UBaseType_t i = 0; i < num_tasks; i++) {
            if (strcmp(task_array[i].pcTaskName, "IDLE0") == 0) {
                *idle0_time = task_array[i].ulRunTimeCounter;
            } else if (strcmp(task_array[i].pcTaskName, "IDLE1") == 0) {
                *idle1_time = task_array[i].ulRunTimeCounter;
            }
        }
        free(task_array);
    }
}

static void start_download_test(void)
{
    ESP_LOGI(TAG, "Starting download test from %s...", TEST_URL);

    // Allocate receive buffer
    char *buf = malloc(RECV_BUF_SIZE);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate receive buffer");
        return;
    }

    esp_http_client_config_t config = {
        .url = TEST_URL,
        .buffer_size = RECV_BUF_SIZE,
        .timeout_ms = 30000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        free(buf);
        return;
    }

    // Open connection and send request
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        free(buf);
        return;
    }

    // Fetch headers to get content length
    int64_t content_length = esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "Connected, status=%d, content_length=%"PRId64, status_code, content_length);

    // Initialize timing and CPU stats
    int64_t start_time = esp_timer_get_time();
    int64_t last_report_time = start_time;
    uint64_t total_bytes = 0;
    uint64_t last_bytes = 0;
    
    configRUN_TIME_COUNTER_TYPE last_total_run_time, last_idle0_time, last_idle1_time;
    get_cpu_idle_times(&last_total_run_time, &last_idle0_time, &last_idle1_time);

    // Read data in a loop
    int read_len;
    while ((read_len = esp_http_client_read(client, buf, RECV_BUF_SIZE)) > 0) {
        total_bytes += read_len;

        // Progress report every PROGRESS_INTERVAL_SEC seconds
        int64_t now = esp_timer_get_time();
        if ((now - last_report_time) >= (PROGRESS_INTERVAL_SEC * 1000000LL)) {
            double elapsed_sec = (now - start_time) / 1000000.0;
            double speed_mbps = (total_bytes * 8.0) / (elapsed_sec * 1000000.0);
            
            // Calculate delta speed for this interval
            uint64_t delta_bytes = total_bytes - last_bytes;
            double interval_sec = (now - last_report_time) / 1000000.0;
            double interval_speed_mbps = (delta_bytes * 8.0) / (interval_sec * 1000000.0);
            
            // Get CPU stats
            configRUN_TIME_COUNTER_TYPE total_run_time, idle0_time, idle1_time;
            get_cpu_idle_times(&total_run_time, &idle0_time, &idle1_time);
            
            // Calculate CPU usage delta
            configRUN_TIME_COUNTER_TYPE delta_total = total_run_time - last_total_run_time;
            configRUN_TIME_COUNTER_TYPE delta_idle0 = idle0_time - last_idle0_time;
            configRUN_TIME_COUNTER_TYPE delta_idle1 = idle1_time - last_idle1_time;
            
            double cpu0_usage = 0, cpu1_usage = 0;
            if (delta_total > 0) {
                cpu0_usage = 100.0 * (1.0 - (double)delta_idle0 / delta_total);
                cpu1_usage = 100.0 * (1.0 - (double)delta_idle1 / delta_total);
                if (cpu0_usage < 0) cpu0_usage = 0;
                if (cpu1_usage < 0) cpu1_usage = 0;
                if (cpu0_usage > 100) cpu0_usage = 100;
                if (cpu1_usage > 100) cpu1_usage = 100;
            }
            
            ESP_LOGI(TAG, "Progress: %.2f MB, avg %.2f Mbps, interval %.2f Mbps, CPU0: %.1f%%, CPU1: %.1f%%",
                     total_bytes / 1000000.0, speed_mbps, interval_speed_mbps, cpu0_usage, cpu1_usage);
            
            // Update last values
            last_report_time = now;
            last_bytes = total_bytes;
            last_total_run_time = total_run_time;
            last_idle0_time = idle0_time;
            last_idle1_time = idle1_time;
        }
    }

    // Final report
    int64_t end_time = esp_timer_get_time();
    double elapsed_sec = (end_time - start_time) / 1000000.0;
    double speed_mbps = (total_bytes * 8.0) / (elapsed_sec * 1000000.0);

    if (read_len == 0) {
        ESP_LOGI(TAG, "Download complete: %.2f MB in %.1fs (%.2f Mbps)",
                 total_bytes / 1000000.0, elapsed_sec, speed_mbps);
    } else {
        ESP_LOGE(TAG, "Download error: %s", esp_err_to_name(read_len));
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(buf);
}

void app_main(void)
{
    ESP_LOGI(TAG, "W6100 Ethernet Example");

    // Create event group
    s_eth_event_group = xEventGroupCreate();

    // Initialize TCP/IP network interface
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default Ethernet netif
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);

    // Get MAC address from ESP32 eFuse (W6100 doesn't have built-in MAC)
    uint8_t mac_addr[6];
    esp_read_mac(mac_addr, ESP_MAC_ETH);
    ESP_LOGI(TAG, "Using MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);

    // Initialize W6100
    w6100_init_config_t w6100_cfg = {
        .spi_host = SPI2_HOST,
        .mosi_gpio = 11,
        .miso_gpio = 13,
        .sclk_gpio = 12,
        .cs_gpio = 10,
        .int_gpio = 14,
        .rst_gpio = 21,
        .spi_clock_mhz = 60,
        .mac_addr = mac_addr,
    };
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(w6100_init(&w6100_cfg, &eth_handle));

    // Attach Ethernet driver to TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               &got_ip_event_handler, NULL));

    // Start Ethernet driver
    ESP_LOGI(TAG, "Starting Ethernet...");
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    ESP_LOGI(TAG, "Waiting for IP address...");

    // Wait for IP address
    xEventGroupWaitBits(s_eth_event_group, ETH_GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // Small delay to let things settle
    vTaskDelay(pdMS_TO_TICKS(500));

    // Start download test
    start_download_test();

    ESP_LOGI(TAG, "Test complete.");
}
