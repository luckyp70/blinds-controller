/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Fortunato Pasqualone.
 *
 * This file is part of the ESP32 Blinds Controller project.
 */

/*
 * Emitted & Handled Events:
 *
 * | Event Name                                         | Emitted | Handled | Description                                      |
 * |----------------------------------------------------|---------|---------|--------------------------------------------------|
 * | APP_EVENT_BLIND_OPENING                            |   No    |   Yes   | Command to open (move up) the blind              |
 * | APP_EVENT_BLIND_CLOSING                            |   No    |   Yes   | Command to close (move down) the blind           |
 * | APP_EVENT_BLIND_STOPPING                           |   No    |   Yes   | Command to stop the blind                        |
 * | APP_EVENT_BLIND_UPDATING_POSITION                  |   No    |   Yes   | Command to move the blind to a specific position |
 * | APP_EVENT_BLIND_STOPPED_AFTER_SWITCH_LIMIT         |  Yes    |   Yes   | Blind stopped after limit switch activation      |
 * | APP_EVENT_BLIND_STOPPED_AFTER_SAFETY_TIME_LIMIT    |  Yes    |   Yes   | Blind stopped after safety time limit            |
 * | APP_EVENT_BLIND_STOPPED                            |  Yes    |   No    | Blind stopped by the user                        |
 * | APP_EVENT_BLIND_MOTOR_STARTED                      |  Yes    |   No    | Motor started (up or down)                       |
 * | APP_EVENT_BLIND_MOTOR_STOPPED                      |  Yes    |   No    | Motor stopped                                    |
 * | APP_EVENT_BLIND_MOTOR_STOPPED_AFTER_SWITCH_LIMIT   |  Yes    |   No    | Motor stopped after limit switch                 |
 * | APP_EVENT_BLIND_MOTOR_STOPPED_AFTER_SAFETY_TIME_LIMIT | Yes |   No    | Motor stopped after safety time limit            |
 *
 * "Emitted" means the module posts the event. "Handled" means the module provides a handler for the event.
 */

/**
 * @file blinds.h
 * @brief Blinds control interface for the ESP32 blinds controller
 *
 * This module manages the high-level blind control logic including
 * position management, calibration, and movement control. It provides
 * an interface for interacting with the blinds system programmatically.
 */

/*
 * Position convention for all APIs and state variables:
 *   - 0% = fully open (blind is completely open)
 *   - 100% = fully closed (blind is completely closed)
 *
 * All position values, including current_position, target_position, and known_position,
 * represent the percentage of closure (not opening!).
 *
 * Example:
 *   - current_position = 0   => blind is fully open
 *   - current_position = 100 => blind is fully closed
 *   - current_position = 50  => blind is half closed
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define FULLY_OPEN_POSITION 0     /**< Fully open position (0% closed) */
#define FULLY_CLOSED_POSITION 100 /**< Fully closed position (100% closed) */
#define HALF_CLOSED_POSITION 50   /**< Half closed position (50% closed) */

#define POSITION_UPDATE_INTERVAL_MS 2000 /**< Interval for updating position (in milliseconds) */

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
 * @brief Stop reason for the blinds
 *
 * Enum representing the reason why a blind
 */
typedef enum
{
    BLIND_STOPPED_NONE = 0,             /**< No stop reason */
    BLIND_STOPPED_BY_USER,              /**< Blind stopped by user */
    BLIND_STOPPED_BY_TIMER,             /**< Blind stopped by timer */
    BLIND_STOPPED_BY_SWITCH_LIMIT,      /**< Blind stopped by limit switch */
    BLIND_STOPPED_BY_SAFETY_TIME_LIMIT, /**< Blind stopped by safety limit */
} blind_stop_reason_t;

typedef struct blind_position_s
{
    blind_id_t blind_id; /**< ID of the blind */
    uint8_t position;    /**< Current position (0-100%). Please notice that it expresses how much it is CLOSED, not open!! */
} blind_position_t;

/**
 * @brief Blind state structure
 *
 * Contains the current state, target position and calibration status.
 * Position convention:
 *   - 0% = fully open
 *   - 100% = fully closed
 */
typedef struct blind_state_s
{
    uint8_t current_position;          /**< Current position (100 = fully closed, 0 = fully open) */
    uint8_t target_position;           /**< Target position to move to (0-100, 100 = fully closed) */
    blind_motion_state_t motion_state; /**< Current motion state */
    uint32_t full_opening_duration;    /**< Duration of the movement in milliseconds */
    uint32_t full_closing_duration;    /**< Duration of the movement in milliseconds */
    bool position_known;               /**< Whether the current position is known */
    bool calibrated_opening;           /**< Whether the blind has been calibrated on opening */
    bool calibrated_closing;           /**< Whether the blind has been calibrated on closing */
} blind_state_t;

/**
 * @brief Initialize the blinds system
 *
 * Sets up initial state and required hardware for all blinds
 */
esp_err_t blinds_init(void);

void blind_open(blind_id_t blind_id);

void blind_close(blind_id_t blind_id);

/**
 * @brief Set target position for a specific blind
 *
 * Position convention:
 *   - 0% = fully open
 *   - 100% = fully closed
 *
 * @param blind_id Which blind to control
 * @param target_position Target position (0-100%)
 */
void blind_move_to_position(blind_id_t blind_id, uint8_t target_position);

/**
 * @brief Stop movement of a specific blind
 *
 * @param blind_id Which blind to stop
 */
void blind_stop(blind_id_t blind_id);

/**
 * @brief Get the current motion state of a specific blind
 *
 * @param blind_id Which blind to query
 * @return Current motion state of the blind
 */
blind_motion_state_t blinds_get_motion_state(blind_id_t blind_id);
