/**
 * @file motors.c
 * @brief Implementation of motor control functionality for blinds controller.
 *
 * This file contains the implementation for controlling two DC motors via H-Bridge drivers
 * for a smart blinds controller. It provides initialization, direction control, safety timeout,
 * and event-driven operation for opening, closing, and stopping blinds.
 */
#include "motors.h"
#include "app_events.h"
#include "mcu.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

static const char *TAG = "motors";

/* GPIO H-Bridge Driver Mapping */
#define MOTOR_1_IN1 CONFIG_BLINDS_CONTROLLER_MOTOR_1_IN1
#define MOTOR_1_IN2 CONFIG_BLINDS_CONTROLLER_MOTOR_1_IN2
#define MOTOR_2_IN1 CONFIG_BLINDS_CONTROLLER_MOTOR_2_IN1
#define MOTOR_2_IN2 CONFIG_BLINDS_CONTROLLER_MOTOR_2_IN2

/* GPIO Motor current reading Mapping */
#define MOTOR_1_CURRENT CONFIG_BLINDS_CONTROLLER_MOTOR_1_CURRENT // 18 --> 00
#define MOTOR_2_CURRENT CONFIG_BLINDS_CONTROLLER_MOTOR_2_CURRENT // 19 --> 01

/* Current sampling period (ms)*/
#define CURRENT_SAMPLING_PERIOD_MS 1000
#define CURRENT_RUNNING_MOTOR_VOLTAGE_THRESHOLD 1.4 // Voltage threshold to consider the motor running
#define MOTOR_CURRENT_SAMPLING_GRACE_PERIOD_MS 500  // Grace period in ms before current sampling is considered valid

typedef struct motor_gpio_s
{
    gpio_num_t in1;
    gpio_num_t in2;
    adc_channel_t current; // Current reading channel
} motor_gpio_t;

/**
 * @brief Internal structure holding all state for a single motor.
 *
 * This struct consolidates the state, timers, and timing information for each motor.
 */
typedef struct motor_internal_s
{
    motor_state_t state;                  /**< Current state of the motor (stopped, moving up, moving down) */
    TimerHandle_t safety_timer;           /**< Safety timeout timer handle */
    TimerHandle_t current_sampling_timer; /**< Current sampling timer handle */
    uint32_t start_tick;                  /**< Tick count when the motor started moving */
} motor_internal_t;

static motor_gpio_t motors_gpio[] = {
    [MOTOR_1] = {.in1 = MOTOR_1_IN1, .in2 = MOTOR_1_IN2, .current = MOTOR_1_CURRENT},
    [MOTOR_2] = {.in1 = MOTOR_2_IN1, .in2 = MOTOR_2_IN2, .current = MOTOR_2_CURRENT},
};

/**
 * @brief Array of per-motor state structures, indexed by motor_id_t.
 */
static motor_internal_t motors[] = {
    [MOTOR_1] = {.state = MOTOR_STOPPED, .safety_timer = NULL, .current_sampling_timer = NULL, .start_tick = 0},
    [MOTOR_2] = {.state = MOTOR_STOPPED, .safety_timer = NULL, .current_sampling_timer = NULL, .start_tick = 0},
};

static portMUX_TYPE motor_mux = portMUX_INITIALIZER_UNLOCKED;

/* Static function declarations */
static void set_motor_direction(motor_id_t motor, bool in1_level, bool in2_level);
static void start_safety_timeout_timer(motor_id_t motor_id);
static void stop_safety_timeout_timer(motor_id_t motor_id);
static void current_sampling_timer_callback(TimerHandle_t xTimer);
static void start_motor_current_sampling(motor_id_t motor_id);
static void stop_motor_current_sampling(motor_id_t motor_id);

/**
 * @brief Initializes the motor driver pins and sets their initial state.
 *
 * Configures the GPIO pins for both motors as outputs and sets them to a safe initial state.
 * Registers event handlers for blind open, close, and stop events.
 *
 * @return ESP_OK on success, or an error code from the ESP-IDF API.
 */
esp_err_t motors_init(void)
{
    ESP_LOGI(TAG, "Initializing motor driver pins...");

    for (int i = 0; i < 2; i++)
    {
        gpio_config_t motor_control_io_conf = {
            .pin_bit_mask = (1ULL << motors_gpio[i].in1) | (1ULL << motors_gpio[i].in2), // Configure both pins for the motor
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        // Configure the GPIO pins for the motor
        ESP_RETURN_ON_ERROR(gpio_config(&motor_control_io_conf), TAG, "Failed to configure GPIO pins to control motor %d", i);

        set_motor_direction(i, false, false); // Set both IN1 and IN2 low
    }
    ESP_LOGI(TAG, "Motor driver pins initialized.");

    return ESP_OK;
}

/**
 * @brief Moves the specified motor up (open blinds).
 *
 * Stops the motor briefly before setting the direction to up (IN1 high, IN2 low),
 * and starts the safety timeout timer.
 *
 * @param motor_id The ID of the motor to move up.
 */
void motor_up(motor_id_t motor_id)
{
    portENTER_CRITICAL(&motor_mux);
    set_motor_direction(motor_id, false, false); // Stop the motor for a moment
    motors[motor_id].state = MOTOR_STOPPED;
    portEXIT_CRITICAL(&motor_mux);
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait to ensure the motor is stopped
    portENTER_CRITICAL(&motor_mux);
    set_motor_direction(motor_id, true, false); // Set IN1 high and IN2 low
    motors[motor_id].state = MOTOR_MOVING_UP;
    motors[motor_id].start_tick = xTaskGetTickCount();
    portEXIT_CRITICAL(&motor_mux);

    start_safety_timeout_timer(motor_id);
    start_motor_current_sampling(motor_id);

    app_event_post(APP_EVENT_BLIND_MOTOR_STARTED, &motor_id, sizeof(motor_id));
}

/**
 * @brief Moves the specified motor down (close blinds).
 *
 * @param motor_id The ID of the motor to move down.
 */
void motor_down(motor_id_t motor_id)
{
    portENTER_CRITICAL(&motor_mux);
    set_motor_direction(motor_id, false, false); // Stop the motor for a moment
    motors[motor_id].state = MOTOR_STOPPED;
    portEXIT_CRITICAL(&motor_mux);
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait to ensure the motor is stopped
    portENTER_CRITICAL(&motor_mux);
    set_motor_direction(motor_id, false, true); // Set IN1 low and IN2 high
    motors[motor_id].state = MOTOR_MOVING_DOWN;
    motors[motor_id].start_tick = xTaskGetTickCount();
    portEXIT_CRITICAL(&motor_mux);

    start_safety_timeout_timer(motor_id);
    start_motor_current_sampling(motor_id);

    app_event_post(APP_EVENT_BLIND_MOTOR_STARTED, &motor_id, sizeof(motor_id));
}

/**
 * @brief Stops the specified motor.
 *
 * @param motor_id The ID of the motor to stop.
 */
void motor_stop(motor_id_t motor_id)
{
    portENTER_CRITICAL(&motor_mux);
    set_motor_direction(motor_id, false, false); // Set both IN1 and IN2 low
    motors[motor_id].state = MOTOR_STOPPED;
    portEXIT_CRITICAL(&motor_mux);

    stop_safety_timeout_timer(motor_id);
    stop_motor_current_sampling(motor_id);

    app_event_post(APP_EVENT_BLIND_MOTOR_STOPPED, &motor_id, sizeof(motor_id));
}

/**
 * @brief Gets the current state of the specified motor.
 *
 * @param motor_id The ID of the motor to query.
 * @return The current state of the motor.
 */
motor_state_t motor_get_state(motor_id_t motor_id)
{
    return motors[motor_id].state;
}

/**
 * @brief Sets the direction of the motor by configuring GPIO levels atomically.
 *
 * @param motor_id The ID of the motor to configure.
 * @param in1_level The level to set for the IN1 pin.
 * @param in2_level The level to set for the IN2 pin.
 */
static void set_motor_direction(motor_id_t motor_id, bool in1_level, bool in2_level)
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
static void safety_timeout_callback(TimerHandle_t xTimer)
{
    motor_id_t motor_id = (motor_id_t)(uintptr_t)pvTimerGetTimerID(xTimer);

    if (motors[motor_id].state == MOTOR_STOPPED)
    {
        return;
    }

    ESP_LOGI(TAG, "Motor %d timed out — stopping", motor_id);
    motor_stop(motor_id);

    app_event_post(APP_EVENT_BLIND_MOTOR_STOPPED_AFTER_SAFETY_TIME_LIMIT, &motor_id, sizeof(motor_id));
}

/**
 * @brief Starts or resets the safety timeout timer for the specified motor.
 *        It's a safety measure to ensure the motor stops after a timeout.
 *
 * @param motor_id The ID of the motor to start the timer for.
 */
static void start_safety_timeout_timer(motor_id_t motor_id)
{
    if (motors[motor_id].safety_timer == NULL)
    {
        motors[motor_id].safety_timer = xTimerCreate(
            "MotorTimeout",
            pdMS_TO_TICKS(CONFIG_BLINDS_CONTROLLER_MOTOR_SAFETY_TIMEOUT),
            pdFALSE,
            (void *)(uintptr_t)motor_id,
            safety_timeout_callback);
    }

    xTimerStop(motors[motor_id].safety_timer, 0);
    xTimerStart(motors[motor_id].safety_timer, 0);
}

/**
 * @brief Stops the timeout timer for the specified motor.
 *        It's a safety measure to ensure the motor stops after a timeout.
 *
 * @param motor_id The ID of the motor to stop the timer for.
 */
static void stop_safety_timeout_timer(motor_id_t motor_id)
{
    if (motors[motor_id].safety_timer != NULL)
    {
        xTimerStop(motors[motor_id].safety_timer, 0);
    }
}

/**
 * @brief Callback function triggered periodically to sample motor current.
 *
 * This function checks the motor's current and stops the motor if the current
 * indicates that the motor has stopped due to a limit switch or other reason.
 *
 * @param xTimer The timer handle.
 */
static void current_sampling_timer_callback(TimerHandle_t xTimer)
{
    motor_id_t motor_id = (motor_id_t)(uintptr_t)pvTimerGetTimerID(xTimer);

    // Check if the grace period since motor start has elapsed
    uint32_t now = xTaskGetTickCount();
    uint32_t elapsed_ms = (now - motors[motor_id].start_tick) * portTICK_PERIOD_MS;
    if (elapsed_ms < MOTOR_CURRENT_SAMPLING_GRACE_PERIOD_MS)
    {
        // Do not sample current yet, allow time for current to stabilize
        return;
    }

    float voltage = 0.0f;
    ESP_RETURN_VOID_ON_ERROR(mcu_get_adc_voltage(&voltage, motors_gpio[motor_id].current), TAG, "Failed to read ADC voltage for motor %d", motor_id);

    if (voltage < CURRENT_RUNNING_MOTOR_VOLTAGE_THRESHOLD)
    {
        ESP_LOGI(TAG, "Motor %d is stopped. Voltage: %.2f V", motor_id, voltage);
        motor_stop(motor_id);
        app_event_post(APP_EVENT_BLIND_MOTOR_STOPPED_AFTER_SWITCH_LIMIT, &motor_id, sizeof(motor_id));
    }
}

/**
 * @brief Starts or resets the current sampling timer for the specified motor.
 *
 * This timer is used to periodically sample the motor current to detect if the motor has stopped due to a limit switch or other reason.
 *
 * @param motor_id The ID of the motor to start the timer for.
 */
static void start_motor_current_sampling(motor_id_t motor_id)
{
    if (motors[motor_id].current_sampling_timer == NULL)
    {
        motors[motor_id].current_sampling_timer = xTimerCreate(
            "MotorCurrentSampling",
            pdMS_TO_TICKS(CURRENT_SAMPLING_PERIOD_MS),
            pdTRUE,
            (void *)(uintptr_t)motor_id,
            current_sampling_timer_callback);
    }

    xTimerStop(motors[motor_id].current_sampling_timer, 0);  // Stop any active timer
    xTimerStart(motors[motor_id].current_sampling_timer, 0); // Start or reset the timer
}

/**
 * @brief Stops the current sampling timer for the specified motor.
 *
 * This function stops the periodic current sampling timer for the given motor.
 *
 * @param motor_id The ID of the motor to stop the timer for.
 */
static void stop_motor_current_sampling(motor_id_t motor_id)
{
    if (motors[motor_id].current_sampling_timer != NULL)
    {
        xTimerStop(motors[motor_id].current_sampling_timer, 0);
    }
}
