/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * W6100 Low-Level Example for ESP32-S3-DevKitC
 *
 * This example demonstrates how to use the lower-level W6100 API
 * (esp_eth_mac_new_w6100 and esp_eth_phy_new_w6100) directly instead
 * of the convenience w6100_init() function. This gives you more control
 * over the initialization process and is similar to how other SPI Ethernet
 * drivers in ESP-IDF are used.
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "w6100.h"

static const char *TAG = "w6100_low_level";

/* Pin definitions - adjust for your hardware */
#define W6100_MOSI_GPIO     11
#define W6100_MISO_GPIO     13
#define W6100_SCLK_GPIO     12
#define W6100_CS_GPIO       10
#define W6100_INT_GPIO      14
#define W6100_RST_GPIO      21
#define W6100_SPI_HOST      SPI2_HOST
#define W6100_SPI_CLOCK_MHZ 33

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
}

/**
 * @brief Initialize GPIO ISR service if not already installed
 */
static esp_err_t init_gpio_isr_service(void)
{
    esp_err_t ret = gpio_install_isr_service(0);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "GPIO ISR handler already installed");
        return ESP_OK;  // Already installed, not an error
    }
    return ret;
}

/**
 * @brief Initialize SPI bus
 */
static esp_err_t init_spi_bus(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = W6100_MOSI_GPIO,
        .miso_io_num = W6100_MISO_GPIO,
        .sclk_io_num = W6100_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    return spi_bus_initialize(W6100_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
}

/**
 * @brief Perform hardware reset on W6100
 */
static void reset_w6100(void)
{
    if (W6100_RST_GPIO >= 0) {
        ESP_LOGI(TAG, "Resetting W6100...");
        gpio_config_t gpio_cfg = {
            .pin_bit_mask = (1ULL << W6100_RST_GPIO),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&gpio_cfg);

        // Assert reset (active low)
        gpio_set_level(W6100_RST_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(10));

        // Release reset
        gpio_set_level(W6100_RST_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(50));  // Wait for chip to stabilize
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "W6100 Low-Level Ethernet Example");

    // Initialize TCP/IP network interface
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default Ethernet netif
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);

    // Perform hardware reset
    reset_w6100();

    // Install GPIO ISR handler (needed for interrupt-driven mode)
    ESP_ERROR_CHECK(init_gpio_isr_service());

    // Initialize SPI bus
    ESP_LOGI(TAG, "Initializing SPI bus...");
    ESP_ERROR_CHECK(init_spi_bus());

    // Configure SPI device interface
    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = W6100_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = W6100_CS_GPIO,
        .queue_size = 20,
        .cs_ena_posttrans = 2,  // Keep CS low slightly after transaction
    };

    // Configure W6100 MAC
    // Note: int_gpio_num and poll_period_ms are mutually exclusive:
    // - For interrupt mode: set int_gpio_num >= 0 and poll_period_ms = 0
    // - For polling mode: set int_gpio_num = -1 and poll_period_ms > 0
    eth_w6100_config_t w6100_config = {
        .int_gpio_num = W6100_INT_GPIO,
        .poll_period_ms = 0,  // Must be 0 when using interrupt mode
        .spi_host_id = W6100_SPI_HOST,
        .spi_devcfg = &spi_devcfg,
        .custom_spi_driver = ETH_DEFAULT_SPI,  // Use default SPI driver
    };

    // Configure MAC
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    mac_config.rx_task_stack_size = 4096;  // Increase if needed

    ESP_LOGI(TAG, "Creating W6100 MAC...");
    esp_eth_mac_t *mac = esp_eth_mac_new_w6100(&w6100_config, &mac_config);
    if (mac == NULL) {
        ESP_LOGE(TAG, "Failed to create W6100 MAC");
        return;
    }

    // Configure PHY
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 1;            // W6100 internal PHY address
    phy_config.reset_gpio_num = -1;     // We already did hardware reset

    ESP_LOGI(TAG, "Creating W6100 PHY...");
    esp_eth_phy_t *phy = esp_eth_phy_new_w6100(&phy_config);
    if (phy == NULL) {
        ESP_LOGE(TAG, "Failed to create W6100 PHY");
        mac->del(mac);
        return;
    }

    // Install Ethernet driver
    ESP_LOGI(TAG, "Installing Ethernet driver...");
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    esp_err_t ret = esp_eth_driver_install(&eth_config, &eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ethernet driver install failed: %s", esp_err_to_name(ret));
        mac->del(mac);
        phy->del(phy);
        return;
    }

    // Get MAC address from ESP32 eFuse (W6100 doesn't have built-in MAC)
    uint8_t mac_addr[6];
    esp_read_mac(mac_addr, ESP_MAC_ETH);
    ESP_LOGI(TAG, "Setting MAC address: %02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);

    ret = esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MAC address: %s", esp_err_to_name(ret));
        esp_eth_driver_uninstall(eth_handle);
        mac->del(mac);
        phy->del(phy);
        return;
    }

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
}
