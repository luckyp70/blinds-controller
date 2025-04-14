#include "buttons.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_err.h"
#include "motors.h"

static const char *TAG = "BUTTONS";

#define BUTTON_T1_DOWN GPIO_NUM_10
#define BUTTON_T1_UP GPIO_NUM_11
#define BUTTON_T2_DOWN GPIO_NUM_14
#define BUTTON_T2_UP GPIO_NUM_15

#define DEBOUNCE_TIME_MS 50

static const gpio_num_t button_gpios[BUTTON_COUNT] = {
    [BUTTON_T1_UP_ID] = BUTTON_T1_UP,
    [BUTTON_T1_DOWN_ID] = BUTTON_T1_DOWN,
    [BUTTON_T2_UP_ID] = BUTTON_T2_UP,
    [BUTTON_T2_DOWN_ID] = BUTTON_T2_DOWN,
};

static TimerHandle_t debounce_timers[BUTTON_COUNT];

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    button_id_t button_id = (button_id_t)(intptr_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Avvia il timer di debounce
    // xTimerStartFromISR(debounce_timers[button_id], &xHigherPriorityTaskWoken);
    xTimerResetFromISR(debounce_timers[button_id], &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

static void debounce_timer_callback(TimerHandle_t xTimer)
{
    button_id_t button_id = (button_id_t)(intptr_t)pvTimerGetTimerID(xTimer);
    gpio_num_t gpio_num = button_gpios[button_id];

    // Verifica lo stato del pulsante
    if (gpio_get_level(gpio_num) == 0)
    {
        ESP_LOGI(TAG, "Button %d pressed", button_id);

        // Determina quale motore e direzione controllare
        motor_id_t motor = (button_id == BUTTON_T1_UP_ID || button_id == BUTTON_T1_DOWN_ID) ? MOTOR_1 : MOTOR_2;
        bool is_up = (button_id == BUTTON_T1_UP_ID || button_id == BUTTON_T2_UP_ID);

        if (is_up)
        {
            motor_toggle_up(motor);
        }
        else
        {
            motor_toggle_down(motor);
        }
    }
}

void buttons_init(void)
{
    ESP_LOGI(TAG, "Initializing buttons...");

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = 0,
    };

    // Configura ciascun pulsante
    for (int i = 0; i < BUTTON_COUNT; i++)
    {
        io_conf.pin_bit_mask = (1ULL << button_gpios[i]);
        gpio_config(&io_conf);

        // Crea il timer di debounce
        debounce_timers[i] = xTimerCreate("debounce_timer", pdMS_TO_TICKS(DEBOUNCE_TIME_MS), pdFALSE, (void *)(intptr_t)i, debounce_timer_callback);
    }

    // Installa il servizio ISR
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Aggiunge gli handler ISR per ciascun pulsante
    for (int i = 0; i < BUTTON_COUNT; i++)
    {
        gpio_isr_handler_add(button_gpios[i], gpio_isr_handler, (void *)(intptr_t)i);
    }

    ESP_LOGI(TAG, "Buttons initialized.");
}