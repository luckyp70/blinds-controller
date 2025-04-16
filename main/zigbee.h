/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Fortunato Pasqualone.
 *
 * This file is part of the ESP32 Blinds Controller project.
 */

/**
 * @file zigbee.h
 * @brief Zigbee interface for ESP32 blinds controller
 *
 * This module handles Zigbee wireless communication for remote control
 * of window coverings, implementing standard ZCL clusters.
 */

#pragma once

#include "esp_zigbee_core.h"
#include "sdkconfig.h"
#include "zcl_utility.h"

/**
 * @brief Configure the Zigbee role based on Kconfig selection
 */
#if CONFIG_BLINDS_CONTROLLER_ZB_ROLE == CONFIG_ZIGBEE_ROLE_ROUTER
// Zigbee Router
#define ZB_ROLE ESP_ZB_DEVICE_TYPE_ROUTER
#elif CONFIG_BLINDS_CONTROLLER_ZB_ROLE == CONFIG_ZIGBEE_ROLE_END_DEVICE
// Zigbee End Device
#define ZB_ROLE ESP_ZB_DEVICE_TYPE_ED
#else
#error "No valid Zigbee role selected in Kconfig"
#endif

/* Zigbee configuration */
/**
 * @brief Enable/disable install code policy for security
 */
#define INSTALLCODE_POLICY_ENABLE CONFIG_BLINDS_CONTROLLER_INSTALLCODE_POLICY_ENABLE

/**
 * @brief End device aging timeout (how long coordinator keeps device in its tables)
 */
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN

/**
 * @brief End device keep-alive interval in milliseconds
 */
#define ED_KEEP_ALIVE CONFIG_BLINDS_CONTROLLER_ED_KEEP_ALIVE /* 3000 millisecond */

/**
 * @brief Window covering endpoint for ZCL commands
 */
#define HA_ESP_WINDOW_COVERING_ENDPOINT 0x0A

/**
 * @brief Channel mask for Zigbee radio
 */
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

/**
 * @brief Manufacturer name for basic cluster
 */
#define ESP_MANUFACTURER_NAME "\x09" \
                              "ESPRESSIF" /* Customized manufacturer name */

/**
 * @brief Model identifier for basic cluster
 */
#define ESP_MODEL_IDENTIFIER "\x07" CONFIG_IDF_TARGET /* Customized model identifier */

/**
 * @brief Zigbee end device configuration macro
 *
 * Configures a Zigbee end device with appropriate settings
 */
// #define ESP_ZB_ZED_CONFIG()   // TODO: Finalize the following macro

#define INSTALLCODE_POLICY_DISABLE 0 // TODO replace in the config below with INSTALLCODE_POLICY_ENABLE

#if CONFIG_BLINDS_CONTROLLER_ZB_ROLE == CONFIG_ZIGBEE_ROLE_ROUTER
#define ESP_ZB_DEVICE_CONFIG()                                                  \
    (esp_zb_cfg_t)                                                              \
    {                                                                           \
        .esp_zb_role = ZB_ROLE,                                                 \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,                       \
        .nwk_cfg.zczr_cfg = { /* puoi aggiungere parametri router se servono */ \
        }                                                                       \
    }
#else
#define ESP_ZB_DEVICE_CONFIG()                             \
    (esp_zb_cfg_t)                                         \
    {                                                      \
        .esp_zb_role = ZB_ROLE,                            \
        .install_code_policy = INSTALLCODE_POLICY_DISABLE, \
        .nwk_cfg.zed_cfg = {                               \
            .ed_timeout = ED_AGING_TIMEOUT,                \
            .keep_alive = ED_KEEP_ALIVE                    \
        }                                                  \
    }
#endif

/**
 * @brief Default radio configuration macro
 *
 * Sets up the radio in native mode
 */
#define ESP_ZB_DEFAULT_RADIO_CONFIG()       \
    {                                       \
        .radio_mode = ZB_RADIO_MODE_NATIVE, \
    }

/**
 * @brief Default host configuration macro
 *
 * Sets up the host connection mode (none for standalone operation)
 */
#define ESP_ZB_DEFAULT_HOST_CONFIG()                          \
    {                                                         \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, \
    }

/**
 * @brief Initialize the Zigbee stack and device
 *
 * Sets up the Zigbee networking, endpoints, and clusters
 */
void zigbee_init(void);
