#include "app_events.h"
#include "esp_log.h"
#include "esp_check.h"

ESP_EVENT_DEFINE_BASE(APP_EVENT);

static const char *TAG = "EVENTS";

/**
 * @brief The event loop handle for the application.
 */
static esp_event_loop_handle_t app_event_loop = NULL;

/**
 * @brief Initialize the application event system by creating the default and application event loops.
 *
 * This function creates the default event loop (if not already created) and a dedicated event loop for application events.
 * It should be called once during application startup.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t app_event_init(void)
{
    // Initialize the default event loop (should be done only once)
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "Failed to create default event loop");

    // Configure and create a dedicated event loop for application events
    esp_event_loop_args_t loop_args = {
        .queue_size = 10,              /**< Maximum number of events in the queue */
        .task_name = "app_event_task", /**< Task name */
        .task_priority = 5,            /**< Task priority */
        .task_stack_size = 4096,       /**< Stack size for the task */
        .task_core_id = 0,             /**< Core ID where the task runs */
    };

    ESP_RETURN_ON_ERROR(esp_event_loop_create(&loop_args, &app_event_loop),
                        TAG,
                        "Failed to create event loop");

    ESP_LOGI(TAG, "Event loop created successfully");
    return ESP_OK;
}

/**
 * @brief Register an event handler for a specific application event.
 *
 * @param event_id           The event ID to register the handler for.
 * @param event_handler      The handler function to be called when the event occurs.
 * @param event_handler_arg  Argument to be passed to the handler function.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t app_event_register(app_event_id_t event_id, esp_event_handler_t event_handler, void *event_handler_arg)
{
    // Register the event handler for the specified event ID
    ESP_RETURN_ON_ERROR(
        esp_event_handler_register_with(
            app_event_loop,
            APP_EVENT,
            event_id,
            event_handler,
            event_handler_arg),
        TAG,
        "Failed to register event handler for the topic %s", app_event_id_to_str(event_id));
    ESP_LOGI(TAG, "Event handler registered for event %s", app_event_id_to_str(event_id));

    return ESP_OK;
}

/**
 * @brief Post an application event to the event loop.
 *
 * @param event_id         The event ID to post.
 * @param event_data       Pointer to the event data.
 * @param event_data_size  Size of the event data.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t app_event_post(app_event_id_t event_id, const void *event_data, size_t event_data_size)
{
    ESP_RETURN_ON_ERROR(
        esp_event_post_to(
            app_event_loop,
            APP_EVENT,
            event_id,
            event_data,
            event_data_size,
            portMAX_DELAY),
        TAG, "Failed to post event %s", app_event_id_to_str(event_id));
    ESP_LOGI(TAG, "Event %s posted successfully", app_event_id_to_str(event_id));

    return ESP_OK;
}

/**
 * @brief Converts an application event ID to its string representation.
 *
 * This function returns a human-readable string corresponding to the given
 * app_event_id_t value. Useful for logging and debugging.
 *
 * @param event_id The event ID to convert.
 * @return The string name of the event, or "UNKNOWN_EVENT" if not recognized.
 */
const char *app_event_id_to_str(app_event_id_t event_id)
{
    switch (event_id)
    {
    case APP_EVENT_NONE:
        return "APP_EVENT_NONE";
    case APP_EVENT_BLIND_OPENING:
        return "APP_EVENT_BLIND_OPENING";
    case APP_EVENT_BLIND_CLOSING:
        return "APP_EVENT_BLIND_CLOSING";
    case APP_EVENT_BLIND_STOPPING:
        return "APP_EVENT_BLIND_STOPPING";
    case APP_EVENT_BLIND_STOPPED:
        return "APP_EVENT_BLIND_STOPPED";
    case APP_EVENT_BLIND_STOPPED_ON_LIMIT:
        return "APP_EVENT_BLIND_STOPPED_ON_LIMIT";
    case APP_EVENT_BLIND_STOPPED_ON_SAFETY_LIMIT:
        return "APP_EVENT_BLIND_STOPPED_ON_SAFETY_LIMIT";
    case APP_EVENT_BLIND_CHANGING_POSITION:
        return "APP_EVENT_BLIND_CHANGING_POSITION";
    case APP_EVENT_BLIND_POSITION_SET:
        return "APP_EVENT_BLIND_POSITION_SET";
    case APP_EVENT_BLIND_CALIBRATING:
        return "APP_EVENT_BLIND_CALIBRATING";
    case APP_EVENT_BLIND_CALIBRATED:
        return "APP_EVENT_BLIND_CALIBRATED";
    default:
        return "UNKNOWN_EVENT";
    }
}
