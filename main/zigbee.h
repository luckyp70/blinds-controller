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
 * of window coverings, implementing standard ZCL clusters. It provides
 * an interface for configuring Zigbee roles and managing Zigbee events.
 */

/*
 * Emitted & Handled Events Recap:
 *
 * | Event Name                        | Emitted | Handled | Description                                  |
 * |-----------------------------------|---------|---------|----------------------------------------------|
 * | APP_EVENT_BLIND_OPENING           |   Yes   |   No    | Zigbee command to open the blind             |
 * | APP_EVENT_BLIND_CLOSING           |   Yes   |   No    | Zigbee command to close the blind            |
 * | APP_EVENT_BLIND_STOPPING          |   Yes   |   No    | Zigbee command to stop the blind             |
 * | APP_EVENT_BLIND_UPDATING_POSITION |   Yes   |   No    | Zigbee command to move to a specific position|
 * | APP_EVENT_BLIND_POSITION_UPDATED  |   No    |   Yes   | Updates Zigbee attribute for blind position  |
 *
 * "Emitted" means the module posts the event. "Handled" means the module provides a handler for the event.
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
#ifdef CONFIG_BLINDS_CONTROLLER_INSTALLCODE_POLICY_ENABLE
#define INSTALLCODE_POLICY_ENABLE 1
#else
#define INSTALLCODE_POLICY_ENABLE 0
#endif

/**
 * @brief End device aging timeout (how long coordinator keeps device in its tables)
 */
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN

/**
 * @brief End device keep-alive interval in milliseconds
 */
#define ED_KEEP_ALIVE CONFIG_BLINDS_CONTROLLER_ED_KEEP_ALIVE /* 3000 millisecond */

#define MANUFACTURER_NAME "\x11" \
                          "LUCKYP TECHNOLOGY"
#define MODEL_IDENTIFIER "\x11" \
                         "Blinds Controller"

/**
 * @brief Channel mask for Zigbee radio
 */
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

/**
 * @brief Zigbee end device configuration macro
 *
 * Configures a Zigbee end device with appropriate settings
 */
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
#define ESP_ZB_DEVICE_CONFIG()                            \
    (esp_zb_cfg_t)                                        \
    {                                                     \
        .esp_zb_role = ZB_ROLE,                           \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
        .nwk_cfg.zed_cfg = {                              \
            .ed_timeout = ED_AGING_TIMEOUT,               \
            .keep_alive = ED_KEEP_ALIVE                   \
        }                                                 \
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

#define BLINDS_ENDPOINT_COUNT 2 /**< Number of window covering endpoints */

/**
 * @brief Endpoint identifiers
 *
 * Enum to identify which endpoint to control in a multi-endpoints setup
 */
typedef enum
{
    BLINDS_ENDPOINT_A = 0x0A, /**< First blind endpoint */
    BLINDS_ENDPOINT_B = 0x0B, /**< Second blind endpoint */
} blind_endpoint_id_t;

/**
 * @brief Initialize the Zigbee stack and device
 *
 * Sets up the Zigbee networking, endpoints, and clusters
 */
esp_err_t zigbee_init(void);
