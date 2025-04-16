/**
 * @file buttons.c
 * @brief Implementation of button handling functionality for blinds controller
 */
#include "buttons.h"
#include "app_events.h"
#include "motors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"

static const char *TAG = "BUTTONS";

/* GPIO pin configurations from SDK config */
#define BUTTON_T1_DOWN CONFIG_BLINDS_CONTROLLER_BUTTON_T1_DOWN
#define BUTTON_T1_UP CONFIG_BLINDS_CONTROLLER_BUTTON_T1_UP
#define BUTTON_T2_DOWN CONFIG_BLINDS_CONTROLLER_BUTTON_T2_DOWN
#define BUTTON_T2_UP CONFIG_BLINDS_CONTROLLER_BUTTON_T2_UP

/* Time in milliseconds for button debounce */
#define DEBOUNCE_TIME_MS 25

/**
 * @brief GPIO mapping for each button
 */
static const gpio_num_t button_gpios[BUTTON_COUNT] = {
    [BUTTON_T1_UP_ID] = BUTTON_T1_UP,
    [BUTTON_T1_DOWN_ID] = BUTTON_T1_DOWN,
    [BUTTON_T2_UP_ID] = BUTTON_T2_UP,
    [BUTTON_T2_DOWN_ID] = BUTTON_T2_DOWN,
};

/**
 * @brief Motor mapping for each button
 */
static const motor_id_t button_to_motor[BUTTON_COUNT] = {
    [BUTTON_T1_UP_ID] = MOTOR_1,
    [BUTTON_T1_DOWN_ID] = MOTOR_1,
    [BUTTON_T2_UP_ID] = MOTOR_2,
    [BUTTON_T2_DOWN_ID] = MOTOR_2,
};

/* Timers for button debounce handling */
static TimerHandle_t debounce_timers[BUTTON_COUNT];

/**
 * @brief ISR handler for button GPIO interrupts
 *
 * This function is called when a button is pressed. It resets the debounce
 * timer to prevent false triggers from button bouncing.
 *
 * @param arg Button ID passed as argument
 */
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    button_id_t button_id = (button_id_t)(intptr_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Reset debounce timer */
    xTimerResetFromISR(debounce_timers[button_id], &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Callback function for button debounce timer
 *
 * Verifies the button state after debounce period and triggers
 * the appropriate motor action if button is still pressed.
 *
 * @param xTimer Timer handle that triggered this callback
 */
static void debounce_timer_callback(TimerHandle_t xTimer)
{
    button_id_t button_id = (button_id_t)(intptr_t)pvTimerGetTimerID(xTimer);
    gpio_num_t gpio_num = button_gpios[button_id];

    /* Check button state after debounce period */
    if (gpio_get_level(gpio_num) == 0)
    {
        ESP_LOGI(TAG, "Button %d pressed", button_id);

        /* Use the button_to_motor map to determine the motor */
        motor_id_t motor_id = button_to_motor[button_id];
        bool is_up = (button_id == BUTTON_T1_UP_ID || button_id == BUTTON_T2_UP_ID);

        app_event_id_t event_id;
        motor_state_t motor_state = motor_get_state(motor_id);

        // Please notice: since there is no physical stop button, we stop the motor
        // when a button is pressed again while the motor is moving in the direction of the button.
        if (is_up)
        {
            event_id = (motor_state == MOTOR_MOVING_UP) ? APP_EVENT_BLIND_STOPPING : APP_EVENT_BLIND_OPENING;
        }
        else
        {
            event_id = (motor_state == MOTOR_MOVING_DOWN) ? APP_EVENT_BLIND_STOPPING : APP_EVENT_BLIND_CLOSING;
        }
        app_event_post(event_id, &motor_id, sizeof(motor_id));
    }
}

/**
 * @brief Initialize button handling functionality
 *
 * Configures GPIO pins, creates debounce timers and installs ISR handlers
 * for all buttons used in the blinds controller.
 *
 */
void buttons_init(void)
{
    ESP_LOGI(TAG, "Initializing buttons...");

    /* Configure GPIO pins for buttons.
     * Internal pull up and down resistors are disabled assuming that debouncing is
     * handled externally (a typical configuration includes a pull up resistor: 122kΩ,
     * a 100nF capacitor between GPIO and the ground and a 22kΩ resistor in series with the button).
     */
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = 0,
    };

    /* Configure each button GPIO */
    for (int i = 0; i < BUTTON_COUNT; i++)
    {
        io_conf.pin_bit_mask = (1ULL << button_gpios[i]);
        gpio_config(&io_conf);

        // Crea il timer di debounce
        debounce_timers[i] = xTimerCreate("debounce_timer", pdMS_TO_TICKS(DEBOUNCE_TIME_MS), pdFALSE, (void *)(intptr_t)i, debounce_timer_callback);
    }

    /* Install ISR service */
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    /* Add ISR handlers for each button */
    for (int i = 0; i < BUTTON_COUNT; i++)
    {
        gpio_isr_handler_add(button_gpios[i], gpio_isr_handler, (void *)i);
    }

    ESP_LOGI(TAG, "Buttons initialized.");
}