/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Fortunato Pasqualone.
 *
 * This file is part of the ESP32 Blinds Controller project.
 */

/**
 * @file buttons.h
 * @brief Button interface for the ESP32 blinds controller
 *
 * This module handles button input detection and processing for
 * controlling the blinds manually via physical buttons.
 */

#pragma once

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
void buttons_init(void);