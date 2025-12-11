/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_eth_phy.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create a PHY instance for W6100
 *
 * @param[in] config PHY configuration
 *
 * @return
 *      - instance: pointer to PHY instance on success
 *      - NULL: on failure
 *
 * @note The W6100 integrates a built-in PHY, so this driver interfaces
 *       with the chip's internal PHY registers via the MAC driver's
 *       phy_reg_read/write mediator functions.
 */
esp_eth_phy_t *esp_eth_phy_new_w6100(const eth_phy_config_t *config);

#ifdef __cplusplus
}
#endif
