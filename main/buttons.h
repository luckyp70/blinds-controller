/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Fortunato Pasqualone.
 *
 * This file is part of the ESP32 Blinds Controller project.
 */

/*
 * Emitted & Handled Events Recap:
 *
 * | Event Name               | Emitted | Handled | Description                        |
 * |--------------------------|---------|---------|------------------------------------|
 * | APP_EVENT_BLIND_OPENING  |   Yes   |   No    | Button triggers blind opening      |
 * | APP_EVENT_BLIND_CLOSING  |   Yes   |   No    | Button triggers blind closing      |
 * | APP_EVENT_BLIND_STOPPING |   Yes   |   No    | Button triggers blind stopping     |
 *
 * "Emitted" means the module posts the event. "Handled" means the module provides a handler for the event.
 */

/**
 * @file buttons.h
 * @brief Button interface for the ESP32 blinds controller
 *
 * This module handles button input detection and processing for
 * controlling the blinds manually via physical buttons.
 */

#pragma once

#include "esp_err.h"

/**
 * @brief Button identifiers
 *
 * Enum to identify each button in the system
 */
typedef enum
{
    BUTTON_T1_UP_ID = 0, /**< Up button for first blind */
    BUTTON_T1_DOWN_ID,   /**< Down button for first blind */
    BUTTON_T2_UP_ID,     /**< Up button for second blind */
    BUTTON_T2_DOWN_ID,   /**< Down button for second blind */
    BUTTON_COUNT         /**< Total number of buttons */
} button_id_t;

/**
 * @brief Initialize the buttons system
 *
 * Sets up GPIO pins, interrupts, and debounce logic for all buttons
 */
esp_err_t buttons_init(void);