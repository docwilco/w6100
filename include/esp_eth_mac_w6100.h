/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_eth_mac.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Custom SPI Driver Configuration for W6100
 *
 * This structure allows users to provide their own SPI driver instead
 * of using the standard ESP-IDF SPI master driver.
 */
typedef struct {
    /**
     * @brief Custom driver context
     */
    void *ctx;

    /**
     * @brief Initialize the custom SPI driver
     *
     * @param[in] ctx Custom driver context
     * @return
     *      - ESP_OK on success
     *      - ESP_FAIL on failure
     */
    esp_err_t (*init)(void *ctx);

    /**
     * @brief Deinitialize the custom SPI driver
     *
     * @param[in] ctx Custom driver context
     * @return
     *      - ESP_OK on success
     *      - ESP_FAIL on failure
     */
    esp_err_t (*deinit)(void *ctx);

    /**
     * @brief Read data from W6100 using custom SPI driver
     *
     * @param[in] ctx Custom driver context
     * @param[in] cmd Command/address phase data (3 bytes: address[15:8], address[7:0], control)
     * @param[in] cmd_len Command length (typically 3 bytes)
     * @param[out] data Buffer to read data into
     * @param[in] data_len Number of bytes to read
     * @return
     *      - ESP_OK on success
     *      - ESP_FAIL on failure
     */
    esp_err_t (*read)(void *ctx, uint32_t cmd, uint32_t cmd_len, uint8_t *data, uint32_t data_len);

    /**
     * @brief Write data to W6100 using custom SPI driver
     *
     * @param[in] ctx Custom driver context
     * @param[in] cmd Command/address phase data (3 bytes: address[15:8], address[7:0], control)
     * @param[in] cmd_len Command length (typically 3 bytes)
     * @param[in] data Data to write
     * @param[in] data_len Number of bytes to write
     * @return
     *      - ESP_OK on success
     *      - ESP_FAIL on failure
     */
    esp_err_t (*write)(void *ctx, uint32_t cmd, uint32_t cmd_len, const uint8_t *data, uint32_t data_len);
} eth_w6100_custom_spi_driver_t;

/**
 * @brief W6100 specific configuration
 */
typedef struct {
    int int_gpio_num;                          /*!< Interrupt GPIO number, set to -1 to use polling */
    uint32_t poll_period_ms;                   /*!< Polling period in ms, only used when int_gpio_num is -1 */
    spi_host_device_t spi_host_id;             /*!< SPI host device ID */
    spi_device_interface_config_t *spi_devcfg; /*!< SPI device configuration */
    eth_w6100_custom_spi_driver_t *custom_spi_driver; /*!< Custom SPI driver (optional, set to NULL to use default) */
} eth_w6100_config_t;

/**
 * @brief Default W6100 specific configuration
 *
 * @param mosi_io  SPI MOSI pin number
 * @param miso_io  SPI MISO pin number
 * @param sclk_io  SPI SCLK pin number
 * @param cs_io    SPI CS pin number
 * @param int_io   Interrupt pin number (-1 to use polling)
 * @param spi_host SPI host device ID
 */
#define ETH_W6100_DEFAULT_CONFIG(mosi_io, miso_io, sclk_io, cs_io, int_io, spi_host) \
    {                                                                       \
        .int_gpio_num = int_io,                                             \
        .poll_period_ms = 10,                                               \
        .spi_host_id = spi_host,                                            \
        .spi_devcfg = &(spi_device_interface_config_t) {                    \
            .mode = 0,                                                      \
            .clock_speed_hz = 33 * 1000 * 1000,                             \
            .spics_io_num = cs_io,                                          \
            .queue_size = 20,                                               \
            .cs_ena_posttrans = 2,                                          \
        },                                                                  \
        .custom_spi_driver = NULL,                                          \
    }

/**
 * @brief Create W6100 Ethernet MAC instance
 *
 * @param[in] w6100_config W6100 specific configuration
 * @param[in] mac_config Ethernet MAC configuration
 *
 * @return
 *      - instance: pointer to MAC instance on success
 *      - NULL: on failure
 *
 * @note The W6100 chip must be connected via SPI interface
 * @note Interrupt mode is recommended for better performance
 */
esp_eth_mac_t *esp_eth_mac_new_w6100(const eth_w6100_config_t *w6100_config, const eth_mac_config_t *mac_config);

#ifdef __cplusplus
}
#endif
