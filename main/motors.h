/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Fortunato Pasqualone.
 *
 * This file is part of the ESP32 Blinds Controller project.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Author: Fortunato Pasqualone
 * Date: April 15, 2025
 */

/*
 * Emitted & Handled Events Recap:
 *
 * | Event Name                              | Emitted | Handled | Description                                       |
 * |-----------------------------------------|---------|---------|---------------------------------------------------|
 * | APP_EVENT_BLIND_MOTOR_STARTED           |   Yes   |   No    | Blind motor has started rotating                  |
 * | APP_EVENT_BLIND_STOPPED_AFTER_SWITCH_LIMIT   |   Yes   |   No    | Blind has stopped by the limit switch             |
 * | APP_EVENT_BLIND_STOPPED_AFTER_SAFETY_TIME_LIMIT |   Yes   |   No    | Blind stopped on safety limit                     |
 *
 * "Emitted" means the module posts the event. "Handled" means the module provides a handler for the event.
 */

/**
 * @file motors.h
 * @brief Motor control interface for blinds controller
 *
 * This module provides functionality to control the motors used in the blinds system.
 * It allows for basic operations like moving up, down, stopping, and toggling the motors.
 */

#pragma once

#include "esp_err.h"

/**
 * @brief Motor identifiers
 *
 * Enum to identify which motor to control in a multi-motor setup
 */
typedef enum
{
    MOTOR_1 = 0, /**< First motor */
    MOTOR_2,     /**< Second motor */
    MOTOR_COUNT, /**< Total number of motors */
} motor_id_t;

/**
 * @brief Motor movement state
 *
 * Enum to represent the current state of a motor
 */
typedef enum
{
    MOTOR_STOPPED = 0, /**< Motor is not moving */
    MOTOR_MOVING_UP,   /**< Motor is moving blinds up */
    MOTOR_MOVING_DOWN, /**< Motor is moving blinds down */
} motor_state_t;

/**
 * @brief Initialize the motors system
 *
 * Sets up hardware, pins, and initial states for all motors
 */
esp_err_t motors_init(void);

/**
 * @brief Start moving the specified motor up
 *
 * @param motor_id The motor to control
 */
void motor_up(motor_id_t motor_id);

/**
 * @brief Start moving the specified motor down
 *
 * @param motor_id The motor to control
 */
void motor_down(motor_id_t motor_id);

/**
 * @brief Stop the specified motor
 *
 * @param motor_id The motor to control
 */
void motor_stop(motor_id_t motor_id);

/**
 * @brief Get the current state of the specified motor
 *
 * @param motor_id The motor to query
 * @return motor_state_t Current state of the motor
 */
motor_state_t motor_get_state(motor_id_t motor_id);