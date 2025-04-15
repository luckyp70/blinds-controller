#include "motors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "hal/gpio_types.h"

static const char *TAG = "MOTORS";

/* GPIO H-Bridge Driver Mapping */
#define MOTOR_1_IN1 CONFIG_BLINDS_CONTROLLER_MOTOR_1_IN1
#define MOTOR_1_IN2 CONFIG_BLINDS_CONTROLLER_MOTOR_1_IN2
#define MOTOR_2_IN1 CONFIG_BLINDS_CONTROLLER_MOTOR_2_IN1
#define MOTOR_2_IN2 CONFIG_BLINDS_CONTROLLER_MOTOR_2_IN2

/* Motor timeout */
#define MOTOR_TIMEOUT_MS 15000

typedef struct
{
    gpio_num_t in1;
    gpio_num_t in2;
} motor_gpio_t;

static motor_gpio_t motors_gpio[] = {
    [MOTOR_1] = {.in1 = MOTOR_1_IN1, .in2 = MOTOR_1_IN2},
    [MOTOR_2] = {.in1 = MOTOR_2_IN1, .in2 = MOTOR_2_IN2},
};

static motor_state_t motor_states[2] = {MOTOR_STOPPED, MOTOR_STOPPED};

static TimerHandle_t motor_timers[2] = {NULL, NULL};

/**
 * @brief Sets the direction of the motor by configuring GPIO levels.
 *
 * @param motor_id The ID of the motor to configure.
 * @param in1_level The level to set for the IN1 pin.
 * @param in2_level The level to set for the IN2 pin.
 */
static void motors_set_direction(motor_id_t motor, bool in1_level, bool in2_level);

/**
 * @brief Starts or resets the timeout timer for the specified motor.
 *
 * @param motor_id The ID of the motor to start the timer for.
 */
static void motor_start_timeout_timer(motor_id_t motor_id);

/**
 * @brief Stops the timeout timer for the specified motor.
 *
 * @param motor_id The ID of the motor to stop the timer for.
 */
static void motor_stop_timeout_timer(motor_id_t motor_id);

/**
 * @brief Initializes the motor driver pins and sets their initial state.
 */
void motors_init(void)
{
    ESP_LOGI(TAG, "Initializing motor driver pins...");

    for (int i = 0; i < 2; i++)
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << motors_gpio[i].in1) | (1ULL << motors_gpio[i].in2), // Configure both pins for the motor
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE};

        // Configure the GPIO pins for the motor
        gpio_config(&io_conf);

        motor_stop(i); // Set initial state of motors to low
    }
    ESP_LOGI(TAG, "Motor driver pins initialized.");
}

/**
 * @brief Moves the specified motor up.
 *
 * @param motor_id The ID of the motor to move up.
 */
void motor_up(motor_id_t motor_id)
{
    ESP_LOGI(TAG, "Moving motor %d up", motor_id);
    motors_set_direction(motor_id, true, false); // Set IN1 high and IN2 low
    motor_states[motor_id] = MOTOR_MOVING_UP;

    motor_start_timeout_timer(motor_id); // Start the timeout timer
}

/**
 * @brief Moves the specified motor down.
 *
 * @param motor_id The ID of the motor to move down.
 */
void motor_down(motor_id_t motor_id)
{
    ESP_LOGI(TAG, "Moving motor %d down", motor_id);
    motors_set_direction(motor_id, false, true); // Set IN1 low and IN2 high
    motor_states[motor_id] = MOTOR_MOVING_DOWN;

    motor_start_timeout_timer(motor_id); // Start the timeout timer
}

/**
 * @brief Stops the specified motor.
 *
 * @param motor_id The ID of the motor to stop.
 */
void motor_stop(motor_id_t motor_id)
{
    ESP_LOGI(TAG, "Stopping motor %d", motor_id);
    motors_set_direction(motor_id, false, false); // Set both IN1 and IN2 low
    motor_states[motor_id] = MOTOR_STOPPED;

    motor_stop_timeout_timer(motor_id); // Stop the timeout timer
}

/**
 * @brief Gets the current state of the specified motor.
 *
 * @param motor_id The ID of the motor to query.
 * @return The current state of the motor.
 */
motor_state_t motor_get_state(motor_id_t motor_id)
{
    return motor_states[motor_id];
}

/**
 * @brief Toggles the motor state between moving up and stopped.
 *
 * @param motor_id The ID of the motor to toggle.
 */
void motor_toggle_up(motor_id_t motor_id)
{
    if (motor_states[motor_id] == MOTOR_MOVING_UP)
    {
        motor_stop(motor_id);
    }
    else
    {
        motor_up(motor_id);
    }
}

/**
 * @brief Toggles the motor state between moving down and stopped.
 *
 * @param motor_id The ID of the motor to toggle.
 */
void motor_toggle_down(motor_id_t motor_id)
{
    if (motor_states[motor_id] == MOTOR_MOVING_DOWN)
    {
        motor_stop(motor_id);
    }
    else
    {
        motor_down(motor_id);
    }
}

/**
 * @brief Sets the direction of the motor by configuring GPIO levels atomically.
 *
 * @param motor_id The ID of the motor to configure.
 * @param in1_level The level to set for the IN1 pin.
 * @param in2_level The level to set for the IN2 pin.
 */
static void motors_set_direction(motor_id_t motor_id, bool in1_level, bool in2_level)
{
    uint32_t mask_set = 0;
    uint32_t mask_clear = 0;

    if (in1_level)
    {
        mask_set |= (1UL << motors_gpio[motor_id].in1); // Add IN1 to the set mask
    }
    else
    {
        mask_clear |= (1UL << motors_gpio[motor_id].in1); // Add IN1 to the clear mask
    }

    if (in2_level)
    {
        mask_set |= (1UL << motors_gpio[motor_id].in2); // Add IN2 to the set mask
    }
    else
    {
        mask_clear |= (1UL << motors_gpio[motor_id].in2); // Add IN2 to the clear mask
    }

    // Atomic operation: clear first, then set
    GPIO.out_w1tc.val = mask_clear; // Clear the specified pins
    GPIO.out_w1ts.val = mask_set;   // Set the specified pins
}

/**
 * @brief Callback function triggered when the motor timeout expires.
 *
 * @param xTimer The timer handle.
 */
static void motor_timeout_callback(TimerHandle_t xTimer)
{
    motor_id_t motor_id = (motor_id_t)(uintptr_t)pvTimerGetTimerID(xTimer); // Retrieve motor ID from timer
    ESP_LOGI(TAG, "Motor %d timed out — stopping", motor_id);
    motor_stop(motor_id); // Stop the motor
}

/**
 * @brief Starts or resets the timeout timer for the specified motor.
 *
 * @param motor_id The ID of the motor to start the timer for.
 */
static void motor_start_timeout_timer(motor_id_t motor_id)
{
    if (motor_timers[motor_id] == NULL)
    {
        // Create a new timer if it doesn't exist
        motor_timers[motor_id] = xTimerCreate("MotorTimeout", pdMS_TO_TICKS(MOTOR_TIMEOUT_MS), pdFALSE, (void *)(uintptr_t)motor_id, motor_timeout_callback);
    }

    xTimerStop(motor_timers[motor_id], 0);  // Stop any active timer
    xTimerStart(motor_timers[motor_id], 0); // Start or reset the timer
}

/**
 * @brief Stops the timeout timer for the specified motor.
 *
 * @param motor_id The ID of the motor to stop the timer for.
 */
static void motor_stop_timeout_timer(motor_id_t motor_id)
{
    if (motor_timers[motor_id] != NULL)
    {
        xTimerStop(motor_timers[motor_id], 0); // Stop the timer
    }
}
