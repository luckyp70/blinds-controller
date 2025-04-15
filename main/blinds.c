/**
 * @file blinds.c
 * @brief Implementation of blinds control functionality
 *
 * This implementation handles blind position management, movement timing,
 * and interfacing with the motor control layer. It uses FreeRTOS timers
 * to manage timed movements between positions.
 */

#include "blinds.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include "motors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_check.h"

/** Module logging tag */
static const char *TAG = "BLINDS";

/**
 * @brief Internal blind control structure
 *
 * Extends the public blind state with implementation details
 */
typedef struct
{
    blind_t state;            /**< Public blind state information */
    TimerHandle_t move_timer; /**< Timer for position-based movement */
} blind_internal_t;

/** Mapping from blind IDs to their corresponding motor IDs */
static const motor_id_t blind_to_motor_map[BLIND_COUNT] = {
    [BLIND_1] = MOTOR_1,
    [BLIND_2] = MOTOR_2,
};

/** Internal state for all blinds */
static blind_internal_t blinds[BLIND_COUNT];

/**
 * @brief Stop motion of a blind and update position
 *
 * @param blind_id ID of the blind to stop
 */
static void stop_motion(blind_id_t blind_id);

/**
 * @brief Timer callback when blind reaches target position
 *
 * @param xTimer Handle of the timer that expired
 */
static void move_timer_callback(TimerHandle_t xTimer)
{
    blind_id_t blind_id = (blind_id_t)pvTimerGetTimerID(xTimer);
    ESP_LOGI(TAG, "Blind %d reached target", blind_id);
    stop_motion(blind_id);
}

/**
 * @brief Initialize the blinds system
 *
 * Sets up initial state for all blinds and creates timers for movement control
 */
void blinds_init(void)
{
    for (uint8_t i = 0; i < BLIND_COUNT; ++i)
    {
        blinds[i].state.current_position = 0;
        blinds[i].state.target_position = 0;
        blinds[i].state.motion = BLIND_STATE_IDLE;
        blinds[i].state.calibrated = false;

        // Create a timer for position-based movement control
        blinds[i].move_timer = xTimerCreate(
            "blind_move_timer",
            pdMS_TO_TICKS(10000), // dummy, will be changed on use
            pdFALSE,              // one-shot timer
            (void *)(uint32_t)i,  // blind ID as timer ID
            move_timer_callback);
    }
}

/**
 * @brief Set target position for a specific blind
 *
 * Calculates the movement duration based on the distance to travel and
 * starts a timer to stop the movement when the target is reached.
 *
 * @param blind_id Which blind to control
 * @param position_percent Target position (0-100%)
 */
void blinds_set_target_position(blind_id_t blind_id, uint8_t position_percent)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);
    ESP_RETURN_VOID_ON_FALSE(position_percent <= 100, TAG, "Invalid position percent: %d", position_percent);

    blind_t *blind = &blinds[blind_id].state;

    ESP_RETURN_VOID_ON_FALSE(blind->calibrated, TAG, "Blind %d not calibrated. Ignoring command", blind_id);

    ESP_RETURN_VOID_ON_FALSE(position_percent != blind->current_position, TAG, "Blind %d already at position %d%%", blind_id, position_percent);

    blind->target_position = position_percent;

    // Calculate movement duration proportional to the distance to travel
    int16_t delta = (int16_t)position_percent - (int16_t)blind->current_position;
    uint32_t duration = (uint32_t)(abs(delta) * CONFIG_BLINDS_CONTROLLER_BLIND_FULL_TRAVEL_TIME_MS / 100);

    if (delta > 0)
    {
        motor_up(blind_to_motor_map[blind_id]);
        blind->motion = BLIND_STATE_MOVING_UP;
    }
    else
    {
        motor_down(blind_to_motor_map[blind_id]);
        blind->motion = BLIND_STATE_MOVING_DOWN;
    }

    // Set timer to stop movement when target is reached
    xTimerChangePeriod(blinds[blind_id].move_timer, pdMS_TO_TICKS(duration), 0);
    xTimerStart(blinds[blind_id].move_timer, 0);

    ESP_LOGI(TAG, "Moving blind %d to %d%% over %" PRIu32 " ms", blind_id, position_percent, duration);
}

/**
 * @brief Stop motion of a blind and update position
 *
 * Updates the current position to match the target and stops the motor
 *
 * @param blind_id ID of the blind to stop
 */
static void stop_motion(blind_id_t blind_id)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);

    blind_t *blind = &blinds[blind_id].state;

    motor_stop(blind_to_motor_map[blind_id]);
    blind->motion = BLIND_STATE_IDLE;
    blind->current_position = blind->target_position;

    ESP_LOGI(TAG, "Blind %d stopped at %d%%", blind_id, blind->current_position);
}

/**
 * @brief Stop movement of a specific blind
 *
 * Stops the movement timer and immediately stops the blind's motion
 *
 * @param blind_id Which blind to stop
 */
void blinds_stop(blind_id_t blind_id)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);

    xTimerStop(blinds[blind_id].move_timer, 0);
    stop_motion(blind_id);
}

/**
 * @brief Process button events for blind control
 *
 * Moves the blind fully open or closed based on button press
 *
 * @param blind_id Which blind is associated with these buttons
 * @param up_pressed Whether the up button is pressed
 * @param down_pressed Whether the down button is pressed
 */
void blinds_handle_button_event(blind_id_t blind_id, bool up_pressed, bool down_pressed)
{
    if (blind_id >= BLIND_COUNT)
        return;

    if (up_pressed)
    {
        blinds_set_target_position(blind_id, 100); // Fully open
    }
    else if (down_pressed)
    {
        blinds_set_target_position(blind_id, 0); // Fully closed
    }
}

/**
 * @brief Force calibration of a blind to a known position
 *
 * Sets the current position to a known value and marks the blind as calibrated
 *
 * @param blind_id Which blind to calibrate
 * @param known_position Current position to set (0-100%)
 */
void blinds_force_calibration(blind_id_t blind_id, uint8_t known_position)
{
    if (blind_id >= BLIND_COUNT || known_position > 100)
        return;

    blinds[blind_id].state.current_position = known_position;
    blinds[blind_id].state.target_position = known_position;
    blinds[blind_id].state.calibrated = true;
    blinds[blind_id].state.motion = BLIND_STATE_IDLE;

    ESP_LOGI(TAG, "Blind %d calibrated at %d%%", blind_id, known_position);
}