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
#include "esp_err.h"

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
    BLIND_MOTION_STATE_IDLE,       /**< Blind is not moving */
    BLIND_MOTION_STATE_MOVING_UP,  /**< Blind is moving upward */
    BLIND_MOTION_STATE_MOVING_DOWN /**< Blind is moving downward */
} blind_motion_state_t;

/**
 * @brief Blind state structure
 *
 * Contains the current state, target position and calibration status
 */
typedef struct
{
    uint8_t current_position;          /**< Current position (0 = fully closed, 100 = fully open) */
    uint8_t target_position;           /**< Target position to move to */
    blind_motion_state_t motion_state; /**< Current motion state */
    uint32_t full_opening_duration;    /**< Duration of the movement in milliseconds */
    uint32_t full_closing_duration;    /**< Duration of the movement in milliseconds */
    bool calibrated;                   /**< Whether the blind has been calibrated */
} blind_t;

/**
 * @brief Initialize the blinds system
 *
 * Sets up initial state and required hardware for all blinds
 */
esp_err_t blinds_init(void);

void blinds_open(blind_id_t blind_id);

void blinds_close(blind_id_t blind_id);

/**
 * @brief Set target position for a specific blind
 *
 * @param blind_id Which blind to control
 * @param position_percent Target position (0-100%)
 */
void blinds_move_to_position(blind_id_t blind_id, uint8_t target_position);

/**
 * @brief Stop movement of a specific blind
 *
 * @param blind_id Which blind to stop
 */
void blinds_stop(blind_id_t blind_id);

blind_motion_state_t blinds_get_motion_state(blind_id_t blind_id);

/**
 * @brief Force calibration of a blind to a known position
 *
 * @param blind_id Which blind to calibrate
 * @param known_position Current position to set (0-100%)
 */
void blinds_force_calibration(blind_id_t blind_id, uint8_t known_position);