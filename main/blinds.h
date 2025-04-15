/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Fortunato Pasqualone.
 *
 * This file is part of the ESP32 Blinds Controller project.
 */

/**
 * @file blinds.h
 * @brief Blinds control interface for the ESP32 blinds controller
 *
 * This module manages the high-level blind control logic including
 * position management, calibration, and movement control.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Blind identifiers
 *
 * Enum to identify which blind to control in a multi-blind setup
 */
typedef enum
{
    BLIND_1 = 0, /**< First blind */
    BLIND_2,     /**< Second blind */
    BLIND_COUNT, /**< Total number of blinds */
} blind_id_t;

/**
 * @brief Motion state of the blinds
 *
 * Enum representing the current movement state of a blind
 */
typedef enum
{
    BLIND_STATE_IDLE,       /**< Blind is not moving */
    BLIND_STATE_MOVING_UP,  /**< Blind is moving upward */
    BLIND_STATE_MOVING_DOWN /**< Blind is moving downward */
} blind_motion_state_t;

/**
 * @brief Blind state structure
 *
 * Contains the current state, target position and calibration status
 */
typedef struct
{
    uint8_t current_position;    /**< Current position (0 = fully closed, 100 = fully open) */
    uint8_t target_position;     /**< Target position to move to */
    blind_motion_state_t motion; /**< Current motion state */
    bool calibrated;             /**< Whether the blind has been calibrated */
} blind_t;

/**
 * @brief Initialize the blinds system
 *
 * Sets up initial state and required hardware for all blinds
 */
void blinds_init(void);

/**
 * @brief Set target position for a specific blind
 *
 * @param blind_id Which blind to control
 * @param position_percent Target position (0-100%)
 */
void blinds_set_target_position(blind_id_t blind_id, uint8_t position_percent);

/**
 * @brief Stop movement of a specific blind
 *
 * @param blind_id Which blind to stop
 */
void blinds_stop(blind_id_t blind_id);

/**
 * @brief Process button events for blind control
 *
 * @param blind_id Which blind is associated with these buttons
 * @param up_pressed Whether the up button is pressed
 * @param down_pressed Whether the down button is pressed
 */
void blinds_handle_button_event(blind_id_t blind_id, bool up_pressed, bool down_pressed);

/**
 * @brief Force calibration of a blind to a known position
 *
 * @param blind_id Which blind to calibrate
 * @param known_position Current position to set (0-100%)
 */
void blinds_force_calibration(blind_id_t blind_id, uint8_t known_position);