/**
 * @file blinds.c
 * @brief Implementation of blinds control functionality
 *
 * This implementation handles blind position management, movement timing,
 * and interfacing with the motor control layer. It uses FreeRTOS timers
 * to manage timed movements between positions.
 */

#include "blinds.h"
#include "app_events.h"
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
static const char *TAG = "BLINDS";

/**
 * @brief Internal blind control structure
 *
 * Extends the public blind state with implementation details
 */
typedef struct blind_internal_s
{
    blind_state_t state;         /**< Public blind state information */
    TimerHandle_t move_timer;    /**< Timer for position-based movement */
    uint32_t move_start_time;    /**< Tick count when movement started */
    uint8_t move_start_position; /**< Position when movement started */
} blind_internal_t;

/** Mapping from blind IDs to their corresponding motor IDs */
static const motor_id_t blind_to_motor[BLIND_COUNT] = {
    [BLIND_1] = MOTOR_1,
    [BLIND_2] = MOTOR_2,
};

static portMUX_TYPE blind_mux = portMUX_INITIALIZER_UNLOCKED;

/** Internal state for all blinds */
static blind_internal_t blinds[BLIND_COUNT];

static void stop_movement_handler(blind_id_t blind_id, bool is_stopped_by_user);
static void stop_timer_callback(TimerHandle_t xTimer);

/* Static function declaration */
static void blind_opening_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void blind_closing_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void blind_moving_to_position_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void blind_stopping_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void move_uncalibrated_blind_to_position(blind_id_t blind_id, uint8_t target_position, uint32_t *duration);
static void move_calibrated_blind_to_position(blind_id_t blind_id, uint8_t target_position, uint32_t *duration);
static void update_position(blind_id_t blind_id, uint8_t current_position);
static esp_err_t blinds_load_calibration(blind_id_t blind_id);
static esp_err_t blinds_save_calibration(blind_id_t blind_id);

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

        if (blinds_load_calibration(i) != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to load calibration for blind %d", i);
            blinds[i].state.full_opening_duration = 12000; // TODO set automatically after calibration
            blinds[i].state.full_closing_duration = 12000; // TODO set automatically after calibration
            blinds[i].state.calibrated = false;
        }

        // Create a timer for position-based movement control
        blinds[i].move_timer = xTimerCreate(
            "blind_move_timer",
            pdMS_TO_TICKS(10000), // dummy, will be changed on use
            pdFALSE,              // one-shot timer
            (void *)(uint32_t)i,  // blind ID as timer ID
            stop_timer_callback);
    }

    // Register event handlers
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_OPENING, &blind_opening_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_OPENING");
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_CLOSING, &blind_closing_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_CLOSING");
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_UPDATING_POSITION, &blind_moving_to_position_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_UPDATING_POSITION");
    ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_STOPPING, &blind_stopping_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_STOPPING");
    ESP_LOGI(TAG, "Event handlers registered.");

    return ESP_OK;
}

/**
 * @brief Open the specified blind fully (move to 100%)
 *
 * Initiates movement to the fully open position (100%).
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
 * @brief Close the specified blind fully (move to 0%)
 *
 * Initiates movement to the fully closed position (0%).
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
 * Calculates the movement duration based on the distance to travel and
 * starts a timer to stop the movement when the target is reached.
 *
 * @param blind_id Which blind to control
 * @param target_position Target position (0-100%)
 */
void blind_move_to_position(blind_id_t blind_id, uint8_t target_position)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);
    ESP_RETURN_VOID_ON_FALSE(target_position <= 100, TAG, "Invalid position percent: %d", target_position);
    const blind_state_t *blind_state = &blinds[blind_id].state;

    uint32_t duration = 0;

    if (!blind_state->calibrated)
    {
        move_uncalibrated_blind_to_position(blind_id, target_position, &duration);
    }
    else
    {
        move_calibrated_blind_to_position(blind_id, target_position, &duration);
    }

    blinds[blind_id].move_start_time = xTaskGetTickCount();
    blinds[blind_id].move_start_position = blind_state->current_position;
    xTimerChangePeriod(blinds[blind_id].move_timer, pdMS_TO_TICKS(duration), 0);
    xTimerStart(blinds[blind_id].move_timer, 0);
    ESP_LOGI(TAG, "Moving blind %d to %d%% over %" PRIu32 " ms", blind_id, target_position, duration);
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
    stop_movement_handler(blind_id, true);
}

/**
 * @brief Update the blind's position when stopping movement.
 *
 * Calculates the new position based on elapsed time and direction.
 *
 * @param blind_state Pointer to the blind state structure
 * @param blind Pointer to the internal blind structure
 * @param is_stopped_by_user True if stopped by user, false if stopped automatically
 */
static void update_position_on_stop(blind_id_t blind_id, bool is_stopped_by_user)
{
    const blind_state_t *blind_state = &blinds[blind_id].state;
    const blind_internal_t *blind = &blinds[blind_id];
    uint8_t current_position;

    if (is_stopped_by_user)
    {
        uint32_t elapsed_ticks = xTaskGetTickCount() - blind->move_start_time;
        uint32_t elapsed_ms = elapsed_ticks * portTICK_PERIOD_MS;
        int16_t delta = (int16_t)blind_state->target_position - (int16_t)blind->move_start_position;
        uint32_t full_duration = (delta > 0) ? blind_state->full_opening_duration : blind_state->full_closing_duration;
        int16_t moved = 0;
        if (full_duration > 0)
        {
            moved = (int16_t)((elapsed_ms * abs(delta)) / full_duration);
        }
        if (delta > 0)
        {
            // blind_state->current_position = (uint8_t)(blind->move_start_position + moved);
            // if (blind_state->current_position > blind_state->target_position)
            //     blind_state->current_position = blind_state->target_position;
            current_position = (uint8_t)(blind->move_start_position + moved);
            if (current_position > blind_state->target_position)
                current_position = blind_state->target_position;
        }

        else
        {
            // blind_state->current_position = (uint8_t)(blind->move_start_position - moved);
            // if (blind_state->current_position < blind_state->target_position)
            //     blind_state->current_position = blind_state->target_position;
            current_position = (uint8_t)(blind->move_start_position - moved);
            if (current_position < blind_state->target_position)
                current_position = blind_state->target_position;
        }
    }
    else
    {
        // blind_state->current_position = blind_state->target_position;
        current_position = blind_state->target_position;
    }
    update_position(blind_id, current_position);
}

/**
 * @brief Move the blind to a target position without calibration.
 *
 * If the blind is not calibrated, only fully open or fully close actions are performed
 * based on the target position (>=50% opens, <50% closes).
 *
 * @param blind_id Blind to move
 * @param target_position Target position (0-100%)
 * @param duration Pointer to store the calculated movement duration (ms)
 */
static void move_uncalibrated_blind_to_position(blind_id_t blind_id, uint8_t target_position, uint32_t *duration)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);
    ESP_RETURN_VOID_ON_FALSE(target_position <= 100, TAG, "Invalid position percent: %d", target_position);
    blind_state_t *blind_state = &blinds[blind_id].state;

    ESP_LOGW(TAG, "Blind %d not calibrated, intermediate positions not managed. Blind will be fully opened (< 50%%) or closed (<= 50%%) based on given position", blind_id);

    portENTER_CRITICAL(&blind_mux);
    if (target_position < FULLY_CLOSED_POSITION / 2)
    {
        motor_up(blind_to_motor[blind_id]);
        blind_state->motion_state = BLIND_MOTION_STATE_MOVING_UP;
        blind_state->target_position = FULLY_OPEN_POSITION;
        *duration = blind_state->full_opening_duration;
    }
    else
    {
        motor_down(blind_to_motor[blind_id]);
        blind_state->motion_state = BLIND_MOTION_STATE_MOVING_DOWN;
        blind_state->target_position = FULLY_CLOSED_POSITION;
        *duration = blind_state->full_closing_duration;
    }
    portEXIT_CRITICAL(&blind_mux);
}

/**
 * @brief Move the blind to a target position with calibration.
 *
 * Calculates the required movement and duration based on calibration data.
 *
 * @param blind_id Blind to move
 * @param target_position Target position (0-100%)
 * @param duration Pointer to store the calculated movement duration (ms)
 */
static void move_calibrated_blind_to_position(blind_id_t blind_id, uint8_t target_position, uint32_t *duration)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);
    ESP_RETURN_VOID_ON_FALSE(target_position <= 100, TAG, "Invalid position percent: %d", target_position);
    blind_state_t *blind_state = &blinds[blind_id].state;

    ESP_RETURN_VOID_ON_FALSE(target_position != blind_state->current_position, TAG, "Blind %d already at position %d%%", blind_id, target_position);

    blind_state->target_position = target_position;
    int16_t delta = (int16_t)target_position - (int16_t)blind_state->current_position;
    bool moving_up = delta > 0;

    portENTER_CRITICAL(&blind_mux);
    if (moving_up)
    {
        motor_up(blind_to_motor[blind_id]);
        blind_state->motion_state = BLIND_MOTION_STATE_MOVING_UP;
        *duration = abs(delta) * blind_state->full_opening_duration / 100;
    }
    else
    {
        motor_down(blind_to_motor[blind_id]);
        blind_state->motion_state = BLIND_MOTION_STATE_MOVING_DOWN;
        *duration = abs(delta) * blind_state->full_closing_duration / 100;
    }
    portEXIT_CRITICAL(&blind_mux);
}

static void update_position(blind_id_t blind_id, uint8_t current_position)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);
    ESP_RETURN_VOID_ON_FALSE(current_position <= 100, TAG, "Invalid position percent: %d", current_position);

    // portENTER_CRITICAL(&blind_mux);
    blinds[blind_id].state.current_position = current_position;
    // portEXIT_CRITICAL(&blind_mux);

    blind_position_t blind_position_update = {
        .blind_id = blind_id,
        .position = current_position,
    };

    app_event_post(APP_EVENT_BLIND_POSITION_UPDATED, &blind_position_update, sizeof(blind_position_update));
}

/**
 * @brief Set the position for an uncalibrated blind when stopping.
 *
 * Sets the position to fully open or fully closed based on last movement direction.
 *
 * @param blind_state Pointer to the blind state structure
 */
static void handle_uncalibrated_stop(blind_id_t blind_id)
{
    const blind_state_t *blind_state = &blinds[blind_id].state;
    if (blind_state->motion_state == BLIND_MOTION_STATE_MOVING_UP)
    {
        // blind_state->current_position = 100;
        update_position(blind_id, 0);
    }
    else if (blind_state->motion_state == BLIND_MOTION_STATE_MOVING_DOWN)
    {
        // blind_state->current_position = 0;
        update_position(blind_id, 100);
    }
}

static void stop_movement_handler(blind_id_t blind_id, bool is_stopped_by_user)
{
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);

    xTimerStop(blinds[blind_id].move_timer, 0);

    blind_state_t *blind_state = &blinds[blind_id].state;

    ESP_RETURN_VOID_ON_FALSE(blind_state->motion_state != BLIND_MOTION_STATE_IDLE, TAG, "Blind %d already stopped", blind_id);

    portENTER_CRITICAL(&blind_mux);
    motor_stop(blind_to_motor[blind_id]);

    if (!blind_state->calibrated)
    {
        handle_uncalibrated_stop(blind_id);
    }
    else
    {
        update_position_on_stop(blind_id, is_stopped_by_user);
    }

    blind_state->motion_state = BLIND_MOTION_STATE_IDLE;
    portEXIT_CRITICAL(&blind_mux);

    ESP_LOGI(TAG, "Blind %d stopped at %d%%", blind_id, blind_state->current_position);

    app_event_post(is_stopped_by_user ? APP_EVENT_BLIND_STOPPED : APP_EVENT_BLIND_STOPPED_ON_LIMIT, &blind_id, sizeof(blind_id));
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
    ESP_RETURN_VOID_ON_FALSE(blind_id < BLIND_COUNT, TAG, "Invalid blind ID: %d", blind_id);
    ESP_RETURN_VOID_ON_FALSE(known_position <= 100, TAG, "Invalid position percent: %d", known_position);

    portENTER_CRITICAL(&blind_mux);
    // blinds[blind_id].state.current_position = known_position;
    update_position(blind_id, known_position);
    blinds[blind_id].state.target_position = known_position;
    blinds[blind_id].state.calibrated = true;
    blinds[blind_id].state.motion_state = BLIND_MOTION_STATE_IDLE;
    portEXIT_CRITICAL(&blind_mux);

    ESP_LOGI(TAG, "Blind %d calibrated at %d%%", blind_id, known_position);

    // Save calibration data to NVS
    if (blinds_save_calibration(blind_id) != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to save calibration data for blind %d", blind_id);
    }
}

static void stop_timer_callback(TimerHandle_t xTimer)
{
    blind_id_t blind_id = (blind_id_t)pvTimerGetTimerID(xTimer);
    ESP_LOGI(TAG, "Blind %d reached the target", blind_id);
    stop_movement_handler(blind_id, false);
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
static void blind_closing_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_RETURN_VOID_ON_FALSE(event_base == APP_EVENT, TAG, "Received event different from APP_EVENT base by the blind_closing_event_handler");
    ESP_RETURN_VOID_ON_FALSE(event_id == APP_EVENT_BLIND_CLOSING, TAG, "Received event different from APP_EVENT_BLIND_CLOSING by the blind_closing_event_handler");

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
static void blind_moving_to_position_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_RETURN_VOID_ON_FALSE(event_base == APP_EVENT, TAG, "Received event different from APP_EVENT base by the blind_moving_to_position_event_handler");
    ESP_RETURN_VOID_ON_FALSE(event_id == APP_EVENT_BLIND_UPDATING_POSITION, TAG, "Received event different from APP_EVENT_BLIND_UPDATING_POSITION by the blind_moving_to_position_event_handler");

    const blind_position_t *blind_position = (const blind_position_t *)event_data;
    ESP_RETURN_VOID_ON_FALSE(blind_position->position <= 100 && blind_position->position >= 0, TAG, "Invalid position percent: %d%%", blind_position->position);

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
static void blind_stopping_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_RETURN_VOID_ON_FALSE(event_base == APP_EVENT, TAG, "Received event different from APP_EVENT base by the blind_stopping_event_handler");
    ESP_RETURN_VOID_ON_FALSE(event_id == APP_EVENT_BLIND_STOPPING, TAG, "Received event different from APP_EVENT_BLIND_STOPPING by the blind_stopping_event_handler");

    const blind_id_t *blind_id = (const blind_id_t *)event_data;
    ESP_RETURN_VOID_ON_FALSE(blind_id != NULL, TAG, "Received event with NULL blind ID");

    ESP_LOGI(TAG, "Stopping event received for blind %d", *blind_id);
    stop_movement_handler(*blind_id, true);
}

/**
 * @brief Load calibration data for a blind from NVS.
 *
 * Reads the full opening/closing durations and calibration status from NVS for the specified blind.
 *
 * @param blind_id Blind to load calibration for
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t blinds_load_calibration(blind_id_t blind_id)
{
    esp_err_t ret = ESP_OK;
    nvs_handle_t nvs;

    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs), TAG, "Failed to open NVS namespace");

    char key_open[20];
    char key_close[20];
    char key_calib[20];
    snprintf(key_open, sizeof(key_open), "b%d_open", blind_id);
    snprintf(key_close, sizeof(key_close), "b%d_close", blind_id);
    snprintf(key_calib, sizeof(key_calib), "b%d_calib", blind_id);

    uint32_t open = 0;
    uint32_t close = 0;
    uint8_t calibrated = 0;
    ESP_GOTO_ON_ERROR(nvs_get_u32(nvs, key_open, &open), finally, TAG, "Failed to get open duration for blind %d", blind_id);
    ESP_GOTO_ON_ERROR(nvs_get_u32(nvs, key_close, &close), finally, TAG, "Failed to get close duration for blind %d", blind_id);
    ESP_GOTO_ON_ERROR(nvs_get_u8(nvs, key_calib, &calibrated), finally, TAG, "Failed to get calibration status for blind %d", blind_id);

    if (open > 0 && close > 0)
    {
        blinds[blind_id].state.full_opening_duration = open;
        blinds[blind_id].state.full_closing_duration = close;
        blinds[blind_id].state.calibrated = calibrated;
    }
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
static esp_err_t blinds_save_calibration(blind_id_t blind_id)
{
    esp_err_t ret = ESP_OK;

    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "Failed to open NVS namespace");

    char key_open[20];
    char key_close[20];
    char key_calib[20];
    snprintf(key_open, sizeof(key_open), "b%d_open", blind_id);
    snprintf(key_close, sizeof(key_close), "b%d_close", blind_id);
    snprintf(key_calib, sizeof(key_calib), "b%d_calib", blind_id);

    ESP_GOTO_ON_ERROR(nvs_set_u32(nvs, key_open, blinds[blind_id].state.full_opening_duration), finally, TAG, "Failed to set open duration for blind %d", blind_id);
    ESP_GOTO_ON_ERROR(nvs_set_u32(nvs, key_close, blinds[blind_id].state.full_closing_duration), finally, TAG, "Failed to set close duration for blind %d", blind_id);
    ESP_GOTO_ON_ERROR(nvs_set_u8(nvs, key_calib, blinds[blind_id].state.calibrated), finally, TAG, "Failed to set calibration status for blind %d", blind_id);
    ESP_GOTO_ON_ERROR(nvs_commit(nvs), finally, TAG, "Failed to commit NVS changes for blind %d", blind_id);

finally:
    nvs_close(nvs);
    return ret;
}