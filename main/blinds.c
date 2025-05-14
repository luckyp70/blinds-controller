/**
 * @file blinds.c
 * @brief Implementation of blinds control functionality
 *
 * Position convention:
 *   - 0% = fully open (blind is completely open)
 *   - 100% = fully closed (blind is completely closed)
 *
 * This implementation handles blind position management, movement timing,
 * and interfacing with the motor control layer. It uses FreeRTOS timers
 * to manage timed movements between positions.
 *
 * The module interacts with the event system to notify and respond to
 * blind-related events, such as position updates and movement commands.
 */

#include "blinds.h"
#include "app_events.h"
#include "mcu.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include "motors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_check.h"

#define NVS_NAMESPACE "blinds"

/** Module logging tag */
static const char *TAG = "blinds";

/**
 * @brief Internal blind control structure
 *
 * Extends the public blind state with implementation details
 */
typedef struct blind_internal_s
{
    blind_state_t state;                  /**< Public blind state information */
    uint32_t move_start_time;             /**< Tick count when movement started */
    uint8_t move_start_position;          /**< Position when movement started */
    uint32_t move_stop_time;              /**< Tick count when movement stopped */
    blind_stop_reason_t last_stop_reason; /**< Last stop reason */
    TimerHandle_t stop_moving_timer;      /**< Timer for position-based movement */
    TimerHandle_t update_position_timer;  /**< Timer for periodic position updates */
} blind_internal_t;

/** Mapping from blind IDs to their corresponding motor IDs */
static const motor_id_t blind_to_motor[BLIND_COUNT] = {
    [BLIND_1] = MOTOR_1,
    [BLIND_2] = MOTOR_2,
};

static portMUX_TYPE blind_mux = portMUX_INITIALIZER_UNLOCKED;

/** Internal state for all blinds */
static blind_internal_t blinds[BLIND_COUNT];

/** Static function declaration */
static void stop_movement_handler(blind_id_t blind_id, blind_stop_reason_t stop_reason);
static void stop_moving_timer_callback(TimerHandle_t xTimer);
static void update_position_timer_callback(TimerHandle_t xTimer);
static void opening_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void closing_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void moving_to_position_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void stopping_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void stopped_on_motor_limit_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void calibration_completed_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void check_and_save_calibration(blind_id_t blind_id, uint8_t start_position, uint32_t duration);
static esp_err_t load_calibration(blind_id_t blind_id);
static esp_err_t save_calibration(blind_id_t blind_id);
static void calibration_led_blink_task(void *pvParameters);

/**
 * @brief Initialize the blinds system
 *
 * Sets up initial state for all blinds and creates timers for movement control
 */
esp_err_t blinds_init(void)
{
    ESP_LOGI(TAG, "Initializing the blinds system");
    for (uint8_t i = 0; i < BLIND_COUNT; ++i)
    {
        blinds[i].state.current_position = 0;
        blinds[i].state.target_position = 0;
        blinds[i].state.motion_state = BLIND_MOTION_STATE_IDLE;
        blinds[i].state.position_known = false;
        blinds[i].state.calibrated_opening = false;
        blinds[i].state.calibrated_closing = false;
        blinds[i].state.full_opening_duration = 0;
        blinds[i].state.full_closing_duration = 0;

        if (load_calibration(i) != ESP_OK)
        {
            ESP_LOGI(TAG, "Unable to load calibration for blind %d. Using default opening and closing durations", i);
            blinds[i].state.full_opening_duration = CONFIG_BLINDS_OPENING_DEFAULT_DURATION;
            blinds[i].state.full_closing_duration = CONFIG_BLINDS_CLOSING_DEFAULT_DURATION;
        }
        else
        {
            ESP_LOGI(TAG, "Loaded calibration for blind %d. Opening duration: %" PRIu32 " ms, Closing duration: %" PRIu32 " ms", i, blinds[i].state.full_opening_duration, blinds[i].state.full_closing_duration);
        }

        // Create a timer for position-based stop movement control
        blinds[i].stop_moving_timer = xTimerCreate(
            "blind_move_timer",
            pdMS_TO_TICKS(10000), // dummy, will be changed on use
            pdFALSE,              // one-shot timer
            (void *)(uint32_t)i,  // blind ID as timer ID
            stop_moving_timer_callback);
        ESP_RETURN_ON_FALSE(blinds[i].stop_moving_timer != NULL, ESP_ERR_NO_MEM, TAG, "Failed to create stop moving timer for blind %d", i);

        // Create a timer for periodical position updates while moving up or down
        blinds[i].update_position_timer = xTimerCreate(
            "update_position_timer",
            pdMS_TO_TICKS(POSITION_UPDATE_INTERVAL_MS), // time period in milliseconds
            pdTRUE,                                     // auto-reload timer
            (void *)(uint32_t)i,                        // blind ID as timer ID
            update_position_timer_callback);
        ESP_RETURN_ON_FALSE(blinds[i].update_position_timer != NULL, ESP_ERR_NO_MEM, TAG, "Failed to create update position timer for blind %d", i);

        blinds[i].move_start_time = 0;
        blinds[i].move_start_position = 0;
        blinds[i].move_stop_time = 0;
        blinds[i].last_stop_reason = BLIND_STOPPED_NONE;
    }

    // Register event handlers
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_OPENING, &opening_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_OPENING");
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_CLOSING, &closing_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_CLOSING");
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_UPDATING_POSITION, &moving_to_position_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_UPDATING_POSITION");
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_STOPPING, &stopping_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_STOPPING");
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_MOTOR_STOPPED_AFTER_SWITCH_LIMIT, &stopped_on_motor_limit_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_MOTOR_STOPPED_AFTER_SWITCH_LIMIT");
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_MOTOR_STOPPED_AFTER_SAFETY_TIME_LIMIT, &stopped_on_motor_limit_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_MOTOR_STOPPED_AFTER_SAFETY_TIME_LIMIT");
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_CALIBRATED, &calibration_completed_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_CALIBRATED");
    ESP_LOGI(TAG, "Event handlers registered.");

    return ESP_OK;
}

/**
 * @brief Open the specified blind fully (move to 0%)
 *
 * Initiates movement to the fully open position (0%).
 *
 * @param blind_id ID of the blind to open
 */
void blind_open(blind_id_t blind_id)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);

    ESP_LOGI(TAG, "Full opening blind %d", blind_id);
    blind_move_to_position(blind_id, FULLY_OPEN_POSITION);
}

/**
 * @brief Close the specified blind fully (move to 100%)
 *
 * Initiates movement to the fully closed position (100%).
 *
 * @param blind_id ID of the blind to close
 */
void blind_close(blind_id_t blind_id)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);

    ESP_LOGI(TAG, "Full closing blind %d", blind_id);
    blind_move_to_position(blind_id, FULLY_CLOSED_POSITION);
}

/**
 * @brief Set target position for a specific blind
 *
 * Position convention:
 *   - 0% = fully open
 *   - 100% = fully closed
 *
 * Calculates the movement duration based on the distance to travel and
 * starts a timer to stop the movement when the target is reached.
 *
 * @param blind_id Which blind to control
 * @param target_position Target position (0-100%)
 */
void blind_move_to_position(blind_id_t blind_id, uint8_t target_position)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);
    ESP_RETURN_VOID_ON_FALSE(target_position <= FULLY_CLOSED_POSITION, TAG, "Invalid position percent: %d", target_position);
    blind_state_t *blind_state = &blinds[blind_id].state;

    uint32_t duration = 0;
    bool move_to_limit = target_position == FULLY_OPEN_POSITION || target_position == FULLY_CLOSED_POSITION;
    uint8_t new_target_position = target_position;
    blind_motion_state_t motion_state;

    if (blind_state->position_known)
    {
        ESP_LOGI(TAG, "Blind %d has a known position", blind_id);

        /* Calculate the duration based on the distance to travel */
        int16_t delta = (int16_t)target_position - (int16_t)blind_state->current_position;
        if (delta > 0)
        {
            motion_state = BLIND_MOTION_STATE_MOVING_DOWN;
            duration = (delta * blind_state->full_closing_duration) / 100;
        }
        else if (delta < 0)
        {
            motion_state = BLIND_MOTION_STATE_MOVING_UP;
            duration = (-delta * blind_state->full_opening_duration) / 100;
        }
        else
        {
            ESP_LOGI(TAG, "Blind %d already at target position %d%%", blind_id, target_position);
            return; // Already at target position
        }
    }
    else
    {
        /* Requested target position is ignored apart from being used to determine the direction. */
        if (target_position >= HALF_CLOSED_POSITION)
        {
            ESP_LOGI(TAG, "Blind %d moving down", blind_id);
            motion_state = BLIND_MOTION_STATE_MOVING_DOWN;
            new_target_position = FULLY_CLOSED_POSITION;
            duration = blind_state->full_closing_duration;
        }
        else
        {
            ESP_LOGI(TAG, "Blind %d moving up", blind_id);
            motion_state = BLIND_MOTION_STATE_MOVING_UP;
            new_target_position = FULLY_OPEN_POSITION;
            duration = blind_state->full_opening_duration;
        }
    }

    portENTER_CRITICAL(&blind_mux);
    if (motion_state == BLIND_MOTION_STATE_MOVING_DOWN)
    {
        motor_down(blind_to_motor[blind_id]);
    }
    else
    {
        motor_up(blind_to_motor[blind_id]);
    }
    blind_state->motion_state = motion_state;
    blind_state->target_position = new_target_position;
    blinds[blind_id].move_start_time = xTaskGetTickCount();
    blinds[blind_id].move_start_position = blind_state->current_position;
    portEXIT_CRITICAL(&blind_mux);

    if (move_to_limit)
    {
        /* Stop a previously set timer */
        xTimerStop(blinds[blind_id].stop_moving_timer, 0);
        ESP_LOGI(TAG, "Blind %d moving to limit position %d%%", blind_id, target_position);
    }
    else
    {
        /** Stop moving timer setup and start */
        xTimerChangePeriod(blinds[blind_id].stop_moving_timer, pdMS_TO_TICKS(duration), 0);
        xTimerStart(blinds[blind_id].stop_moving_timer, 0);

        ESP_LOGI(TAG, "Moving blind %d to %d%% over %" PRIu32 " ms", blind_id, target_position, duration);
    }

    /** Update position timer start */
    xTimerStart(blinds[blind_id].update_position_timer, 0);
}

/**
 * @brief Stop movement of a specific blind
 *
 * Stops the movement timer and immediately stops the blind's motion
 *
 * @param blind_id Which blind to stop
 */
void blind_stop(blind_id_t blind_id)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);

    ESP_LOGI(TAG, "Stopping blind %d", blind_id);
    stop_movement_handler(blind_id, BLIND_STOPPED_BY_USER);
}

/**
 * @brief Update the current position of a blind and notify via event system.
 *
 * @param blind_id The blind to update
 * @param current_position The current position to set
 */
static void update_and_notify_position(blind_id_t blind_id, uint8_t current_position)
{
    // Update the current position and notify via event system
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);
    ESP_RETURN_VOID_ON_FALSE(current_position <= FULLY_CLOSED_POSITION, TAG, "Invalid position percent: %d", current_position);

    blinds[blind_id].state.current_position = current_position;

    blind_position_t blind_position_update = {
        .blind_id = blind_id,
        .position = current_position,
    };

    app_event_post(APP_EVENT_BLIND_POSITION_UPDATED, &blind_position_update, sizeof(blind_position_update));
}

/**
 * @brief Interpolates the current position of a blind based on elapsed time and motion state.
 *
 * Position convention:
 *   - 0% = fully open
 *   - 100% = fully closed
 *
 * @param blind Pointer to the internal blind structure
 * @param elapsed_ms Elapsed time in milliseconds since movement started
 * @return Interpolated current position (0-100%)
 */
static uint8_t interpolate_position(const blind_internal_t *blind, uint32_t elapsed_ms)
{
    // Calculate interpolated position based on direction and elapsed time
    uint8_t start_position = blind->move_start_position;
    uint8_t target_position = blind->state.target_position;
    uint32_t full_duration = 0;
    int16_t moved = 0;
    uint8_t current_position = start_position;

    if (blind->state.motion_state == BLIND_MOTION_STATE_MOVING_DOWN)
    {
        full_duration = blind->state.full_closing_duration;
        if (full_duration > 0)
        {
            moved = (int16_t)((elapsed_ms * ((int16_t)target_position - (int16_t)start_position)) / full_duration);
            current_position = (uint8_t)(start_position + moved);
            if (current_position > target_position)
                current_position = target_position;
        }
    }
    else if (blind->state.motion_state == BLIND_MOTION_STATE_MOVING_UP)
    {
        full_duration = blind->state.full_opening_duration;
        if (full_duration > 0)
        {
            moved = (int16_t)((elapsed_ms * ((int16_t)start_position - (int16_t)target_position)) / full_duration);
            current_position = (uint8_t)(start_position - moved);
            if (current_position < target_position)
                current_position = target_position;
        }
    }
    return current_position;
}

/**
 * @brief Get the current position of a blind when stopped by user.
 *
 * @param blind_id The blind to query
 * @return Estimated current position
 */
static uint8_t get_current_position_on_user_stop(blind_id_t blind_id)
{
    const blind_internal_t *blind = &blinds[blind_id];
    uint8_t current_position;
    if (blind->state.position_known)
    {
        uint32_t elapsed_ms = (xTaskGetTickCount() - blind->move_start_time) * portTICK_PERIOD_MS;
        current_position = interpolate_position(blind, elapsed_ms);
    }
    else
    {
        // Since the position is not known and the blind didn't reach the target position, we set the position in the middle
        current_position = HALF_CLOSED_POSITION;
    }
    return current_position;
}

/**
 * @brief Handles stopping the movement of a blind, updates state and notifies events.
 *
 * @param blind_id The blind to stop
 * @param stop_reason The reason for stopping (user, timer, or limit). Refer to blind_stop_reason_t.
 */
static void stop_movement_handler(blind_id_t blind_id, blind_stop_reason_t stop_reason)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);

    blind_internal_t *blind = &blinds[blind_id];

    xTimerStop(blind->stop_moving_timer, 0);
    xTimerStop(blind->update_position_timer, 0);

    ESP_RETURN_VOID_ON_FALSE(blind->state.motion_state != BLIND_MOTION_STATE_IDLE, TAG, "Blind %d already stopped", blind_id);

    blinds[blind_id].move_stop_time = xTaskGetTickCount();

    ESP_LOGI(TAG, "Blind %d stopped while moving %s", blind_id, blind->state.motion_state == BLIND_MOTION_STATE_MOVING_UP ? "up" : "down");

    switch (stop_reason)
    {
    case BLIND_STOPPED_BY_USER:
        ESP_LOGI(TAG, "Blind %d stopped by user", blind_id);
        uint8_t current_position = get_current_position_on_user_stop(blind_id);

        portENTER_CRITICAL(&blind_mux);
        update_and_notify_position(blind_id, current_position);
        blind->state.target_position = current_position;
        motor_stop(blind_to_motor[blind_id]);
        blind->state.motion_state = BLIND_MOTION_STATE_IDLE;
        portEXIT_CRITICAL(&blind_mux);

        app_event_post(APP_EVENT_BLIND_STOPPED, &blind_id, sizeof(blind_id));
        break;

    case BLIND_STOPPED_BY_TIMER:
        ESP_LOGI(TAG, "Blind %d stopped by timer", blind_id);
        portENTER_CRITICAL(&blind_mux);
        update_and_notify_position(blind_id, blind->state.target_position);
        motor_stop(blind_to_motor[blind_id]);
        blind->state.motion_state = BLIND_MOTION_STATE_IDLE;
        // Do NOT set position_known here
        portEXIT_CRITICAL(&blind_mux);

        app_event_post(APP_EVENT_BLIND_STOPPED_AFTER_TIME_LIMIT, &blind_id, sizeof(blind_id));
        break;

    case BLIND_STOPPED_BY_SWITCH_LIMIT:
        ESP_LOGI(TAG, "Blind %d stopped by hard limit", blind_id);
        portENTER_CRITICAL(&blind_mux);
        uint32_t elapsed_ms = (blinds[blind_id].move_stop_time - blinds[blind_id].move_start_time) * portTICK_PERIOD_MS;
        uint8_t start_position = blinds[blind_id].move_start_position;
        update_and_notify_position(blind_id, blind->state.motion_state == BLIND_MOTION_STATE_MOVING_UP ? FULLY_OPEN_POSITION : FULLY_CLOSED_POSITION);
        blind->state.motion_state = BLIND_MOTION_STATE_IDLE;
        blind->state.position_known = true;
        portEXIT_CRITICAL(&blind_mux);

        app_event_post(APP_EVENT_BLIND_STOPPED_AFTER_SWITCH_LIMIT, &blind_id, sizeof(blind_id));

        check_and_save_calibration(blind_id, start_position, elapsed_ms);

        break;

    case BLIND_STOPPED_BY_SAFETY_TIME_LIMIT:
        ESP_LOGI(TAG, "Blind %d stopped by safety limit", blind_id);
        portENTER_CRITICAL(&blind_mux);
        update_and_notify_position(blind_id, blind->state.motion_state == BLIND_MOTION_STATE_MOVING_UP ? FULLY_OPEN_POSITION : FULLY_CLOSED_POSITION); // Assuming it reached the end
        blind->state.motion_state = BLIND_MOTION_STATE_IDLE;
        // Do NOT set position_known here
        portEXIT_CRITICAL(&blind_mux);

        app_event_post(APP_EVENT_BLIND_STOPPED_AFTER_SAFETY_TIME_LIMIT, &blind_id, sizeof(blind_id));
        break;
    default:
        ESP_LOGW(TAG, "Blind %d stopped with unknown reason", blind_id);
        break;
    }
}

/**
 * @brief Get the current motion state of a blind.
 *
 * @param blind_id Blind to query
 * @return The current motion state
 */
blind_motion_state_t blinds_get_motion_state(blind_id_t blind_id)
{
    return blinds[blind_id].state.motion_state;
}

static void stop_moving_timer_callback(TimerHandle_t xTimer)
{
    blind_id_t blind_id = (blind_id_t)pvTimerGetTimerID(xTimer);
    ESP_LOGI(TAG, "Blind %d reached the target", blind_id);
    stop_movement_handler(blind_id, BLIND_STOPPED_BY_TIMER);
}

/**
 * @brief Callback function for the position update timer
 */
static void update_position_timer_callback(TimerHandle_t xTimer)
{
    blind_id_t blind_id = (blind_id_t)pvTimerGetTimerID(xTimer);
    if (blinds[blind_id].state.motion_state != BLIND_MOTION_STATE_IDLE)
    {
        uint32_t elapsed_ms = (xTaskGetTickCount() - blinds[blind_id].move_start_time) * portTICK_PERIOD_MS;
        uint8_t current_position = interpolate_position(&blinds[blind_id], elapsed_ms);

        portENTER_CRITICAL(&blind_mux);
        update_and_notify_position(blind_id, current_position);
        portEXIT_CRITICAL(&blind_mux);
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
static void opening_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_RETURN_VOID_ON_FALSE(event_base == APP_EVENT, TAG, "Received event different from APP_EVENT base by the opening_event_handler");
    ESP_RETURN_VOID_ON_FALSE(event_id == APP_EVENT_BLIND_OPENING, TAG, "Received event different from APP_EVENT_BLIND_OPENING by the opening_event_handler");

    const blind_id_t *blind_id = (const blind_id_t *)event_data;
    ESP_RETURN_VOID_ON_FALSE(blind_id != NULL, TAG, "Received event with NULL blind ID");

    ESP_LOGI(TAG, "Opening event received for blind %d", *blind_id);
    blind_open(*blind_id);
}

/**
 * @brief Event handler for blind closing events. Moves the motor down.
 *
 * @param arg Pointer to the motor ID (as void*).
 * @param event_base Event base (should be APP_EVENT).
 * @param event_id Event ID (should be APP_EVENT_BLIND_CLOSING).
 * @param event_data Pointer to event data (unused).
 */
static void closing_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_RETURN_VOID_ON_FALSE(event_base == APP_EVENT, TAG, "Received event different from APP_EVENT base by the closing_event_handler");
    ESP_RETURN_VOID_ON_FALSE(event_id == APP_EVENT_BLIND_CLOSING, TAG, "Received event different from APP_EVENT_BLIND_CLOSING by the closing_event_handler");

    const blind_id_t *blind_id = (const blind_id_t *)event_data;
    ESP_RETURN_VOID_ON_FALSE(blind_id != NULL, TAG, "Received event with NULL blind ID");

    ESP_LOGI(TAG, "Closing event received for blind %d", *blind_id);
    blind_close(*blind_id);
}

/**
 * @brief Event handler for blind moving-to-position events.
 *
 * Handles the APP_EVENT_BLIND_UPDATING_POSITION event by moving the specified blind to the requested target position.
 * Extracts the target position from event_data and calls blind_move_to_position.
 *
 * @param arg Pointer to the blind ID (as void*).
 * @param event_base Event base (should be APP_EVENT).
 * @param event_id Event ID (should be APP_EVENT_BLIND_UPDATING_POSITION).
 * @param event_data Pointer to event data (should be uint8_t target position).
 */
static void moving_to_position_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_RETURN_VOID_ON_FALSE(event_base == APP_EVENT, TAG, "Received event different from APP_EVENT base by the moving_to_position_event_handler");
    ESP_RETURN_VOID_ON_FALSE(event_id == APP_EVENT_BLIND_UPDATING_POSITION, TAG, "Received event different from APP_EVENT_BLIND_UPDATING_POSITION by the moving_to_position_event_handler");

    const blind_position_t *blind_position = (const blind_position_t *)event_data;
    ESP_RETURN_VOID_ON_FALSE(blind_position->position <= FULLY_CLOSED_POSITION && blind_position->position >= FULLY_OPEN_POSITION, TAG, "Invalid position percent: %d%%", blind_position->position);

    ESP_LOGI(TAG, "Moving to %d%% position event received for blind %d", blind_position->position, blind_position->blind_id);
    blind_move_to_position(blind_position->blind_id, blind_position->position);
}

/**
 * @brief Event handler for blind stopping events. Stops the motor.
 *
 * @param arg Pointer to the motor ID (as void*).
 * @param event_base Event base (should be APP_EVENT).
 * @param event_id Event ID (should be APP_EVENT_BLIND_STOPPING).
 * @param event_data Pointer to event data (unused).
 */
static void stopping_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_RETURN_VOID_ON_FALSE(event_base == APP_EVENT, TAG, "Received event different from APP_EVENT base by the stopping_event_handler");
    ESP_RETURN_VOID_ON_FALSE(event_id == APP_EVENT_BLIND_STOPPING, TAG, "Received event different from APP_EVENT_BLIND_STOPPING by the stopping_event_handler");

    const blind_id_t *blind_id = (const blind_id_t *)event_data;
    ESP_RETURN_VOID_ON_FALSE(blind_id != NULL, TAG, "Received event with NULL blind ID");

    ESP_LOGI(TAG, "Stopping event received for blind %d", *blind_id);
    stop_movement_handler(*blind_id, BLIND_STOPPED_BY_USER);
}

/**
 * @brief Event handler for motor related stopping events. Updates the blind status.
 *
 * @param arg Pointer to the motor ID (as void*).
 * @param event_base Event base (should be APP_EVENT).
 * @param event_id Event ID (should be APP_EVENT_BLIND_STOPPED_AFTER_SWITCH_LIMIT or APP_EVENT_BLIND_STOPPED_ON_SAFETY_LIMIT).
 * @param event_data Pointer to event data (unused).
 */
static void stopped_on_motor_limit_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_RETURN_VOID_ON_FALSE(event_base == APP_EVENT, TAG, "Received event different from APP_EVENT base by the blind_stopping_on_hard_limit_event_handler");
    ESP_RETURN_VOID_ON_FALSE(event_id == APP_EVENT_BLIND_MOTOR_STOPPED_AFTER_SWITCH_LIMIT || event_id == APP_EVENT_BLIND_MOTOR_STOPPED_AFTER_SAFETY_TIME_LIMIT, TAG, "Received unexpected event by the stopped_on_motor_limit_event_handler");

    const blind_id_t *blind_id = (const blind_id_t *)event_data;
    ESP_RETURN_VOID_ON_FALSE(blind_id != NULL, TAG, "Received event with NULL blind ID");

    ESP_LOGI(TAG, "Stopped on motor limit event %ld received for blind %d", event_id, *blind_id);
    stop_movement_handler(*blind_id, event_id == APP_EVENT_BLIND_MOTOR_STOPPED_AFTER_SWITCH_LIMIT ? BLIND_STOPPED_BY_SWITCH_LIMIT : BLIND_STOPPED_BY_SAFETY_TIME_LIMIT);
}

static void calibration_completed_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_RETURN_VOID_ON_FALSE(event_base == APP_EVENT, TAG, "Received event different from APP_EVENT base by the calibration_completed_event_handler");
    ESP_RETURN_VOID_ON_FALSE(event_id == APP_EVENT_BLIND_CALIBRATED, TAG, "Received unexpected event by the calibration_completed_event_handler");

    // const blind_id_t *blind_id = (const blind_id_t *)event_data;
    // ESP_RETURN_VOID_ON_FALSE(blind_id != NULL, TAG, "Received event with NULL blind ID");

    // Blink internal RGB LED blue 5 times asynchronously to indicate calibration complete
    xTaskCreate(
        &calibration_led_blink_task,
        "calib_led_blink",
        1024,
        NULL,
        tskIDLE_PRIORITY,
        NULL);
}

/**
 * @brief Check and save calibration data for a blind.
 *
 * If the blind is already calibrated, do nothing. Otherwise, save the calibration data
 * to NVS if the duration is valid and both opening and closing calibrations are completed.
 *
 * @param blind_id Blind to check and save calibration for
 * @param start_position Starting position (0 = fully open, 100 = fully closed)
 * @param duration Duration of the movement in milliseconds
 */
static void check_and_save_calibration(blind_id_t blind_id, uint8_t start_position, uint32_t duration)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);
    if (!blinds[blind_id].state.position_known)
    {
        ESP_LOGD(TAG, "Calibration attempt for blind %d ignored: position not known", blind_id);
        return;
    }
    if (blinds[blind_id].state.calibrated_opening && blinds[blind_id].state.calibrated_closing)
    {
        // Already calibrated, do nothing
        return;
    }
    ESP_RETURN_VOID_ON_FALSE(duration > 0, TAG, "Invalid duration: %" PRIu32, duration);

    bool calibration_completed = false;

    portENTER_CRITICAL(&blind_mux);
    if (start_position == FULLY_OPEN_POSITION && blinds[blind_id].state.calibrated_closing == false)
    {
        blinds[blind_id].state.full_closing_duration = duration;
        blinds[blind_id].state.calibrated_closing = true;
        calibration_completed = blinds[blind_id].state.calibrated_opening;
    }
    else if (start_position == FULLY_CLOSED_POSITION && blinds[blind_id].state.calibrated_opening == false)
    {
        blinds[blind_id].state.full_opening_duration = duration;
        blinds[blind_id].state.calibrated_opening = true;
        calibration_completed = blinds[blind_id].state.calibrated_closing;
    }
    portEXIT_CRITICAL(&blind_mux);

    if (calibration_completed)
    {
        ESP_RETURN_VOID_ON_ERROR(save_calibration(blind_id), TAG, "Failed to save calibration data for blind %d", blind_id);
        ESP_LOGI(TAG, "Blind %d: calibration complete and saved to NVS", blind_id);

        app_event_post(APP_EVENT_BLIND_CALIBRATED, &blind_id, sizeof(blind_id));
    }
}

/**
 * @brief Load calibration data for a blind from NVS.
 *
 * Reads the full opening/closing durations and calibration status from NVS for the specified blind.
 *
 * @param blind_id Blind to load calibration for
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t load_calibration(blind_id_t blind_id)
{
    esp_err_t ret = ESP_OK;
    nvs_handle_t nvs;

    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs), TAG, "Failed to open NVS namespace");

    char key_open[20];
    char key_close[20];
    char key_calib_opening[20];
    char key_calib_closing[20];

    snprintf(key_open, sizeof(key_open), "b%d_open", blind_id);
    snprintf(key_close, sizeof(key_close), "b%d_close", blind_id);
    snprintf(key_calib_opening, sizeof(key_calib_opening), "b%d_calib_op", blind_id);
    snprintf(key_calib_closing, sizeof(key_calib_closing), "b%d_calib_cl", blind_id);

    uint32_t open = 0;
    uint32_t close = 0;
    uint8_t calibrated_opening = 0;
    uint8_t calibrated_closing = 0;
    ESP_GOTO_ON_ERROR(nvs_get_u32(nvs, key_open, &open), finally, TAG, "Failed to get open duration for blind %d", blind_id);
    ESP_GOTO_ON_ERROR(nvs_get_u32(nvs, key_close, &close), finally, TAG, "Failed to get close duration for blind %d", blind_id);
    ESP_GOTO_ON_ERROR(nvs_get_u8(nvs, key_calib_opening, &calibrated_opening), finally, TAG, "Failed to get calibration opening status for blind %d", blind_id);
    ESP_GOTO_ON_ERROR(nvs_get_u8(nvs, key_calib_closing, &calibrated_closing), finally, TAG, "Failed to get calibration closing status for blind %d", blind_id);

    ESP_GOTO_ON_FALSE(calibrated_opening != 0, ESP_ERR_INVALID_STATE, finally, TAG, "Invalid opening calibration status for blind %d", blind_id);
    ESP_GOTO_ON_FALSE(open > 0, ESP_ERR_INVALID_STATE, finally, TAG, "Invalid opening duration for blind %d", blind_id);
    ESP_GOTO_ON_FALSE(calibrated_closing != 0, ESP_ERR_INVALID_STATE, finally, TAG, "Invalid closing calibration status for blind %d", blind_id);
    ESP_GOTO_ON_FALSE(close > 0, ESP_ERR_INVALID_STATE, finally, TAG, "Invalid closing duration for blind %d", blind_id);

    portENTER_CRITICAL(&blind_mux);
    blinds[blind_id].state.full_opening_duration = open;
    blinds[blind_id].state.calibrated_opening = (calibrated_opening != 0);
    blinds[blind_id].state.full_closing_duration = close;
    blinds[blind_id].state.calibrated_closing = (calibrated_closing != 0);
    portEXIT_CRITICAL(&blind_mux);

finally:
    nvs_close(nvs);
    return ret;
}

/**
 * @brief Save calibration data for a blind to NVS.
 *
 * Stores the full opening/closing durations and calibration status to NVS for the specified blind.
 *
 * @param blind_id Blind to save calibration for
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t save_calibration(blind_id_t blind_id)
{
    esp_err_t ret = ESP_OK;

    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "Failed to open NVS namespace");

    const blind_state_t *blind_state = &blinds[blind_id].state;

    ESP_RETURN_ON_FALSE(blind_state->calibrated_opening, ESP_ERR_INVALID_STATE, TAG, "Invalid opening calibration status for blind %d", blind_id);
    ESP_RETURN_ON_FALSE(blind_state->full_opening_duration > 0, ESP_ERR_INVALID_STATE, TAG, "Invalid opening duration for blind %d", blind_id);
    ESP_RETURN_ON_FALSE(blind_state->calibrated_closing, ESP_ERR_INVALID_STATE, TAG, "Invalid closing calibration status for blind %d", blind_id);
    ESP_RETURN_ON_FALSE(blind_state->full_closing_duration > 0, ESP_ERR_INVALID_STATE, TAG, "Invalid closing duration for blind %d", blind_id);

    char key_open[20];
    char key_close[20];
    char key_calib_opening[20];
    char key_calib_closing[20];

    snprintf(key_open, sizeof(key_open), "b%d_open", blind_id);
    snprintf(key_close, sizeof(key_close), "b%d_close", blind_id);
    snprintf(key_calib_opening, sizeof(key_calib_opening), "b%d_calib_op", blind_id);
    snprintf(key_calib_closing, sizeof(key_calib_closing), "b%d_calib_cl", blind_id);

    ESP_GOTO_ON_ERROR(nvs_set_u32(nvs, key_open, blinds[blind_id].state.full_opening_duration), finally, TAG, "Failed to set open duration for blind %d", blind_id);
    ESP_GOTO_ON_ERROR(nvs_set_u8(nvs, key_calib_opening, blinds[blind_id].state.calibrated_opening), finally, TAG, "Failed to set calibration opening status for blind %d", blind_id);
    ESP_GOTO_ON_ERROR(nvs_set_u32(nvs, key_close, blinds[blind_id].state.full_closing_duration), finally, TAG, "Failed to set close duration for blind %d", blind_id);
    ESP_GOTO_ON_ERROR(nvs_set_u8(nvs, key_calib_closing, blinds[blind_id].state.calibrated_closing), finally, TAG, "Failed to set calibration closing status for blind %d", blind_id);

    ESP_GOTO_ON_ERROR(nvs_commit(nvs), finally, TAG, "Failed to commit NVS changes for blind %d", blind_id);

finally:
    nvs_close(nvs);
    return ret;
}

static void calibration_led_blink_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Calibration LED blink task started");
    mcu_blink_rgb_led(0, 0, 150, 1500, (float)0.6, 5);
    vTaskDelete(NULL);
}