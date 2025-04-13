#include "motors.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "hal/gpio_types.h"

static const char *TAG = "MOTORS";

/* GPIO Mapping */
#define MOTOR_1_IN1 GPIO_NUM_4
#define MOTOR_1_IN2 GPIO_NUM_5
#define MOTOR_2_IN1 GPIO_NUM_6
#define MOTOR_2_IN2 GPIO_NUM_7

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

static void motors_set_direction(motor_id_t motor, bool in1_level, bool in2_level);

void motors_init(void)
{
    ESP_LOGI(TAG, "Initializing motor driver pins...");

    for (int i = 0; i < 2; i++)
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << motors_gpio[i].in1) | (1ULL << motors_gpio[i].in2),
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

void motor_up(motor_id_t motor_id)
{
    ESP_LOGI(TAG, "Moving motor %d up", motor_id);
    // gpio_set_level(motors_gpio[motor_id].in1, 1);
    // gpio_set_level(motors_gpio[motor_id].in2, 0);
    motors_set_direction(motor_id, true, false);
    motor_states[motor_id] = MOTOR_MOVING_UP;
}

void motor_down(motor_id_t motor_id)
{
    ESP_LOGI(TAG, "Moving motor %d down", motor_id);
    // gpio_set_level(motors_gpio[motor_id].in1, 0);
    // gpio_set_level(motors_gpio[motor_id].in2, 1);
    motors_set_direction(motor_id, false, true);
    motor_states[motor_id] = MOTOR_MOVING_DOWN;
}

void motor_stop(motor_id_t motor_id)
{
    ESP_LOGI(TAG, "Stopping motor %d", motor_id);
    // gpio_set_level(motors_gpio[motor_id].in1, 0);
    // gpio_set_level(motors_gpio[motor_id].in2, 0);
    motors_set_direction(motor_id, false, false);
    motor_states[motor_id] = MOTOR_STOPPED;
}

motor_state_t motor_get_state(motor_id_t motor_id)
{
    return motor_states[motor_id];
}

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

static void motors_set_direction(motor_id_t motor_id, bool in1_level, bool in2_level)
{
    uint32_t mask_set = 0;
    uint32_t mask_clear = 0;

    if (in1_level)
    {
        mask_set |= (1UL << motors_gpio[motor_id].in1);
    }
    else
    {
        mask_clear |= (1UL << motors_gpio[motor_id].in1);
    }

    if (in2_level)
    {
        mask_set |= (1UL << motors_gpio[motor_id].in2);
    }
    else
    {
        mask_clear |= (1UL << motors_gpio[motor_id].in2);
    }

    // Operazione atomica: prima clear, poi set
    GPIO.out_w1tc.val = mask_clear;
    GPIO.out_w1ts.val = mask_set;
}