#include "zigbee.h"
#include <string.h>

#include "esp_log.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_window_covering.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ZIGBEE";

/**
 * @brief Initializes the driver logic in a deferred manner.
 *
 * @return ESP_OK if the driver is initialized successfully, ESP_FAIL otherwise.
 */
static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;
    if (!is_inited)
    {
        // TODO: Adapt logic for initializing the driver
        is_inited = true;
    }
    return is_inited ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Callback to start top-level commissioning for Zigbee.
 *
 * @param mode_mask The commissioning mode mask.
 */
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

/**
 * @brief Handles Zigbee application signals.
 *
 * @param signal_struct The Zigbee application signal structure.
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
            }
        }
        else
        {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

/**
 * @brief Handles Zigbee attribute updates.
 *
 * @param message The Zigbee attribute update message.
 * @return ESP_OK if the attribute is handled successfully, an error code otherwise.
 */
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);

    if (message->info.dst_endpoint == HA_ESP_WINDOW_COVERING_ENDPOINT)
    {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING)
        {
            switch (message->attribute.id)
            {
            case ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID:
                ESP_LOGI(TAG, "Window covering position set to %d%%", *(uint8_t *)message->attribute.data.value);
                // TODO: Add logic to move the motor to the specified position
                break;
            default:
                ESP_LOGW(TAG, "Unsupported attribute ID: 0x%x", message->attribute.id);
                break;
            }
        }
    }

    return ret;
}

/**
 * @brief Handles Zigbee actions and commands.
 *
 * @param callback_id The Zigbee action callback ID.
 * @param message The Zigbee action message.
 * @return ESP_OK if the action is handled successfully, an error code otherwise.
 */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID:
    {
        const esp_zb_zcl_custom_cluster_command_message_t *cmd_message = (const esp_zb_zcl_custom_cluster_command_message_t *)message;
        ESP_LOGI(TAG, "Received command: cluster(0x%x), command(0x%x)", cmd_message->info.cluster, cmd_message->info.command.id);

        if (cmd_message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING)
        {
            switch (cmd_message->info.command.id)
            {
            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN:
                ESP_LOGI(TAG, "Window covering UP/Open command received");
                // TODO: Add logic to move the motor up
                break;
            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
                ESP_LOGI(TAG, "Window covering DOWN/Close command received");
                // TODO: Add logic to move the motor down
                break;
            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP:
                ESP_LOGI(TAG, "Window covering STOP command received");
                // TODO: Add logic to stop the motor
                break;
            default:
                ESP_LOGW(TAG, "Unsupported command ID: 0x%x", cmd_message->info.command.id);
                break;
            }
        }
    }
    break;
    default:
        ESP_LOGW(TAG, "Received unhandled Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

/**
 * @brief Zigbee task to initialize and start the Zigbee stack.
 *
 * @param pvParameters Task parameters (unused).
 */
static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_DEVICE_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Configure Window Covering cluster */
    esp_zb_window_covering_cfg_t window_covering_cfg = ESP_ZB_DEFAULT_WINDOW_COVERING_CONFIG();

    /* Create the endpoint for Window Covering */
    esp_zb_ep_list_t *esp_zb_window_covering_ep = esp_zb_window_covering_ep_create(HA_ESP_WINDOW_COVERING_ENDPOINT, &window_covering_cfg);

    /* Add manufacturer info */
    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };
    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_window_covering_ep, HA_ESP_WINDOW_COVERING_ENDPOINT, &info);

    /* Register endpoint and handlers */
    esp_zb_device_register(esp_zb_window_covering_ep);
    esp_zb_core_action_handler_register(zb_action_handler);

    /* Set Zigbee network configuration */
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    /* Start Zigbee stack */
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

/**
 * @brief Initializes the Zigbee stack and starts the Zigbee task.
 */
void zigbee_init(void)
{
    // Initialize Zigbee
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Zigbee initialized");
}
