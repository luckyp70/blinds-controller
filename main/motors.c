/**
 * @file motors.c
 * @brief Implementation of motor control functionality for blinds controller
 */
#include "motors.h"
#include "app_events.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

static const char *TAG = "MOTORS";

/* GPIO H-Bridge Driver Mapping */
#define MOTOR_1_IN1 CONFIG_BLINDS_CONTROLLER_MOTOR_1_IN1
#define MOTOR_1_IN2 CONFIG_BLINDS_CONTROLLER_MOTOR_1_IN2
#define MOTOR_2_IN1 CONFIG_BLINDS_CONTROLLER_MOTOR_2_IN1
#define MOTOR_2_IN2 CONFIG_BLINDS_CONTROLLER_MOTOR_2_IN2

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

static portMUX_TYPE motor_mux = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief Set the direction of the motor by configuring GPIO levels.
 *
 * @param motor_id The ID of the motor to configure.
 * @param in1_level The level to set for the IN1 pin.
 * @param in2_level The level to set for the IN2 pin.
 */
static void motors_set_direction(motor_id_t motor, bool in1_level, bool in2_level);

/**
 * @brief Start or reset the timeout timer for the specified motor.
 *
 * @param motor_id The ID of the motor to start the timer for.
 */
static void motor_start_timeout_timer(motor_id_t motor_id);

/**
 * @brief Stop the timeout timer for the specified motor.
 *
 * @param motor_id The ID of the motor to stop the timer for.
 */
static void motor_stop_safety_timeout_timer(motor_id_t motor_id);

/**
 * @brief Event handler for blind opening events.
 *
 * @param arg Pointer to the motor ID (as void*).
 * @param event_base Event base (should be APP_EVENT).
 * @param event_id Event ID (should be APP_EVENT_BLIND_OPENING).
 * @param event_data Pointer to event data (unused).
 */
static void blind_opening_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

/**
 * @brief Event handler for blind closing events.
 *
 * @param arg Pointer to the motor ID (as void*).
 * @param event_base Event base (should be APP_EVENT).
 * @param event_id Event ID (should be APP_EVENT_BLIND_CLOSING).
 * @param event_data Pointer to event data (unused).
 */
static void blind_closing_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

/**
 * @brief Event handler for blind stopping events.
 *
 * @param arg Pointer to the motor ID (as void*).
 * @param event_base Event base (should be APP_EVENT).
 * @param event_id Event ID (should be APP_EVENT_BLIND_STOPPING).
 * @param event_data Pointer to event data (unused).
 */
static void blind_stopping_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

/**
 * @brief Initializes the motor driver pins and sets their initial state.
 */
esp_err_t motors_init(void)
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
        ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Failed to configure GPIO pins for motor %d", i);

        // motor_stop(i);                         // Set initial state of motors to low
        motors_set_direction(i, false, false); // Set both IN1 and IN2 low
    }
    ESP_LOGI(TAG, "Motor driver pins initialized.");

    // Register event handlers for motor events
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_OPENING, &blind_opening_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_OPENING");
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_CLOSING, &blind_closing_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_CLOSING");
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_STOPPING, &blind_stopping_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_STOPPING");
    ESP_LOGI(TAG, "Event handlers registered.");

    return ESP_OK;
}

/**
 * @brief Moves the specified motor up (open blinds).
 *
 * @param motor_id The ID of the motor to move up.
 */
void motor_up(motor_id_t motor_id)
{
    ESP_LOGI(TAG, "Moving motor %d up", motor_id);
    portENTER_CRITICAL(&motor_mux);
    motors_set_direction(motor_id, true, false); // Set IN1 high and IN2 low
    motor_states[motor_id] = MOTOR_MOVING_UP;
    portEXIT_CRITICAL(&motor_mux);

    motor_start_timeout_timer(motor_id); // Start the timeout timer
}

/**
 * @brief Moves the specified motor down (close blinds).
 *
 * @param motor_id The ID of the motor to move down.
 */
void motor_down(motor_id_t motor_id)
{
    ESP_LOGI(TAG, "Moving motor %d down", motor_id);
    portENTER_CRITICAL(&motor_mux);
    motors_set_direction(motor_id, false, true); // Set IN1 low and IN2 high
    motor_states[motor_id] = MOTOR_MOVING_DOWN;
    portEXIT_CRITICAL(&motor_mux);

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
    portENTER_CRITICAL(&motor_mux);
    motors_set_direction(motor_id, false, false); // Set both IN1 and IN2 low
    motor_states[motor_id] = MOTOR_STOPPED;
    portEXIT_CRITICAL(&motor_mux);

    // Call timer stop outside the critical section
    motor_stop_safety_timeout_timer(motor_id); // Stop the timeout timer
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

    app_event_post(APP_EVENT_BLIND_STOPPED_ON_SAFETY_LIMIT, &motor_id, sizeof(motor_id)); // Post event indicating the motor has stopped
}

/**
 * @brief Starts or resets the timeout timer for the specified motor.
 *        It's a safety measure to ensure the motor stops after a timeout.
 *
 * @param motor_id The ID of the motor to start the timer for.
 */
static void motor_start_timeout_timer(motor_id_t motor_id)
{
    if (motor_timers[motor_id] == NULL)
    {
        // Create a new timer if it doesn't exist
        motor_timers[motor_id] = xTimerCreate(
            "MotorTimeout",
            pdMS_TO_TICKS(CONFIG_BLINDS_CONTROLLER_MOTOR_SAFETY_TIMEOUT),
            pdFALSE,
            (void *)(uintptr_t)motor_id,
            motor_timeout_callback);
    }

    xTimerStop(motor_timers[motor_id], 0);  // Stop any active timer
    xTimerStart(motor_timers[motor_id], 0); // Start or reset the timer
}

/**
 * @brief Stops the timeout timer for the specified motor.
 *        It's a safety measure to ensure the motor stops after a timeout.
 *
 * @param motor_id The ID of the motor to stop the timer for.
 */
static void motor_stop_safety_timeout_timer(motor_id_t motor_id)
{
    if (motor_timers[motor_id] != NULL)
    {
        xTimerStop(motor_timers[motor_id], 0); // Stop the timer
    }
}

/**
 * @brief Event handler for blind opening events. Moves the motor up.
 *
 * @param arg Pointer to the motor ID (as void*).
 * @param event_base Event base (should be APP_EVENT).
 * @param event_id Event ID (should be APP_EVENT_BLIND_OPENING).
 * @param event_data Pointer to event data (unused).
 */
static void blind_opening_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_RETURN_VOID_ON_FALSE(event_base == APP_EVENT, TAG, "Received event different from APP_EVENT base by the blind_opening_event_handler");

    ESP_RETURN_VOID_ON_FALSE(event_id == APP_EVENT_BLIND_OPENING, TAG, "Received event different from APP_EVENT_BLIND_OPENING by the blind_opening_event_handler");

    motor_id_t motor_id = (motor_id_t)(intptr_t)arg;
    ESP_LOGI(TAG, "Blind opening event received for motor %d", motor_id);
    motor_up(motor_id);
}

/**
 * @brief Event handler for blind closing events. Moves the motor down.
 *
 * @param arg Pointer to the motor ID (as void*).
 * @param event_base Event base (should be APP_EVENT).
 * @param event_id Event ID (should be APP_EVENT_BLIND_CLOSING).
 * @param event_data Pointer to event data (unused).
 */
static void blind_closing_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_RETURN_VOID_ON_FALSE(event_base == APP_EVENT, TAG, "Received event different from APP_EVENT base by the blind_closing_event_handler");

    ESP_RETURN_VOID_ON_FALSE(event_id == APP_EVENT_BLIND_CLOSING, TAG, "Received event different from APP_EVENT_BLIND_CLOSING by the blind_closing_event_handler");

    motor_id_t motor_id = (motor_id_t)(intptr_t)arg;
    ESP_LOGI(TAG, "Blind closing event received for motor %d", motor_id);
    motor_down(motor_id);
}

/**
 * @brief Event handler for blind stopping events. Stops the motor.
 *
 * @param arg Pointer to the motor ID (as void*).
 * @param event_base Event base (should be APP_EVENT).
 * @param event_id Event ID (should be APP_EVENT_BLIND_STOPPING).
 * @param event_data Pointer to event data (unused).
 */
static void blind_stopping_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_RETURN_VOID_ON_FALSE(event_base == APP_EVENT, TAG, "Received event different from APP_EVENT base by the blind_stopping_event_handler");

    ESP_RETURN_VOID_ON_FALSE(event_id == APP_EVENT_BLIND_STOPPING, TAG, "Received event different from APP_EVENT_BLIND_STOPPING by the blind_stopping_event_handler");

    motor_id_t motor_id = (motor_id_t)(intptr_t)arg;
    ESP_LOGI(TAG, "Blind stopping event received for motor %d", motor_id);
    motor_stop(motor_id);

    app_event_post(APP_EVENT_BLIND_STOPPED, &motor_id, sizeof(motor_id)); // Post event indicating the motor has stopped
}
