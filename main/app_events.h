#pragma once

#include "esp_event.h"
#include "esp_err.h"

ESP_EVENT_DECLARE_BASE(APP_EVENT);

/**
 * @brief Application event IDs for the blinds controller.
 */
typedef enum app_event_id_t
{
    /* Null Event */
    APP_EVENT_NONE,

    /* BLIND/MOTOR Events */
    APP_EVENT_BLIND_OPENING,                 /**< Blind is opening */
    APP_EVENT_BLIND_CLOSING,                 /**< Blind is closing */
    APP_EVENT_BLIND_STOPPING,                /**< Blind is stopping */
    APP_EVENT_BLIND_STOPPED,                 /**< Blind has stopped */
    APP_EVENT_BLIND_STOPPED_ON_LIMIT,        /**< Blind stopped on limit */
    APP_EVENT_BLIND_STOPPED_ON_SAFETY_LIMIT, /**< Blind stopped on safety limit */
    APP_EVENT_BLIND_UPDATING_POSITION,       /**< Blind is updating its position */
    APP_EVENT_BLIND_POSITION_UPDATED,        /**< Blind position updated */
    APP_EVENT_BLIND_CALIBRATING,             /**< Blind is calibrating */
    APP_EVENT_BLIND_CALIBRATED,              /**< Blind has been calibrated */

} app_event_id_t;

/**
 * @brief Get the string name of an app_event_id_t.
 *
 * This function returns a human-readable string corresponding to the given
 * app_event_id_t value. Useful for logging and debugging.
 *
 * @param event_id The event ID.
 * @return The string name of the event, or "UNKNOWN_EVENT" if not recognized.
 */
const char *app_event_id_to_str(app_event_id_t event_id);

/**
 * @brief Initialize the application event system and create the event loop.
 */
esp_err_t app_event_init(void);

/**
 * @brief Register an event handler for a specific application event.
 *
 * @param event_id           The event ID to register the handler for.
 * @param event_handler      The handler function to be called when the event occurs.
 * @param event_handler_arg  Argument to be passed to the handler function.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t app_event_register(app_event_id_t event_id, esp_event_handler_t event_handler, void *event_handler_arg);

/**
 * @brief Post an application event to the event loop.
 *
 * @param event_id         The event ID to post.
 * @param event_data       Pointer to the event data.
 * @param event_data_size  Size of the event data.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t app_event_post(app_event_id_t event_id, const void *event_data, size_t event_data_size);
