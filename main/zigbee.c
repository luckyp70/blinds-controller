/**
 * @file zigbee.c
 * @brief Implementation of Zigbee interface for ESP32 blinds controller.
 *
 * This file contains the implementation of the Zigbee stack initialization, device discovery,
 * binding, attribute reporting, and event handling for window covering (blinds) control.
 * It integrates with the application event system and blinds driver to provide Zigbee-based
 * automation and remote control of window coverings.
 */

#include "zigbee.h"
#include "app_events.h"
#include "blinds.h"
#include <string.h>

#include "esp_log.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_window_covering.h"

#define ARRAY_LENTH(arr) (sizeof(arr) / sizeof(arr[0]))

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ZIGBEE";

typedef struct zbstring_s
{
    uint8_t len;
    char data[];
} ESP_ZB_PACKED_STRUCT
    zbstring_t;

typedef struct window_cover_device_params_s
{
    esp_zb_ieee_addr_t ieee_addr;
    uint8_t endpoint;
    uint16_t short_addr;
} window_cover_device_params_t;

window_cover_device_params_t window_cover_device;

static uint8_t current_position_lift_percentage = 0;

/* Static function declarations */
static void blind_position_change_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

/**
 * @brief Callback to start top-level commissioning for Zigbee.
 *
 * @param mode_mask The commissioning mode mask.
 */
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_VOID_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, TAG, "Failed to start Zigbee commissioning");
}

/**
 * @brief Callback for Zigbee device binding result.
 *
 * @param zdo_status The status of the binding operation.
 * @param user_ctx Pointer to the user context (binding request parameters).
 */
static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *)user_ctx;

    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS)
    {
        /* Local binding succeeds */
        if (bind_req->req_dst_addr == esp_zb_get_short_address())
        {
            ESP_LOGI(TAG, "Successfully bind the window cover device from address(0x%x) on endpoint(%d)",
                     window_cover_device.short_addr, window_cover_device.endpoint);

            /* Read peer Manufacture Name & Model Identifier */
            esp_zb_zcl_read_attr_cmd_t read_req = {0};
            read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            read_req.zcl_basic_cmd.src_endpoint = BLINDS_WINDOW_COVERING_ENDPOINT_A;
            read_req.zcl_basic_cmd.dst_endpoint = window_cover_device.endpoint;
            read_req.zcl_basic_cmd.dst_addr_u.addr_short = window_cover_device.short_addr;
            read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_BASIC;

            uint16_t attributes[] = {
                ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
            };
            read_req.attr_number = ARRAY_LENTH(attributes);
            read_req.attr_field = attributes;

            esp_zb_zcl_read_attr_cmd_req(&read_req);
        }
        if (bind_req->req_dst_addr == window_cover_device.short_addr)
        {
            ESP_LOGI(TAG, "The window cover device from address(0x%x) on endpoint(%d) successfully binds us",
                     window_cover_device.short_addr, window_cover_device.endpoint);
        }
        free(bind_req);
    }
    else
    {
        /* Bind failed, maybe retry the binding ? */

        // esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);
    }
}

/**
 * @brief Callback for Zigbee device discovery (Match Descriptor response).
 *
 * @param zdo_status The status of the discovery operation.
 * @param peer_addr The short address of the discovered device.
 * @param peer_endpoint The endpoint of the discovered device.
 * @param user_ctx Pointer to the user context (device parameters).
 */
static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t peer_addr, uint8_t peer_endpoint, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS)
    {
        ESP_LOGI(TAG, "Found window cover device");
        /* Store the information of the remote device */
        window_cover_device_params_t *device = (window_cover_device_params_t *)user_ctx;
        device->endpoint = peer_endpoint;
        device->short_addr = peer_addr;
        esp_zb_ieee_address_by_short(device->short_addr, device->ieee_addr);

        /* 1. Send binding request to the sensor */
        esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *)calloc(sizeof(esp_zb_zdo_bind_req_param_t), 1);
        bind_req->req_dst_addr = peer_addr;

        /* populate the src information of the binding */
        memcpy(bind_req->src_address, device->ieee_addr, sizeof(esp_zb_ieee_addr_t));
        bind_req->src_endp = peer_endpoint;
        bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING; // ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;

        /* populate the dst information of the binding */
        bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        esp_zb_get_long_address(bind_req->dst_address_u.addr_long);
        bind_req->dst_endp = BLINDS_WINDOW_COVERING_ENDPOINT_A;

        ESP_LOGI(TAG, "Request window device to bind us");
        esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);

        /* 2. Send binding request to self */
        bind_req = (esp_zb_zdo_bind_req_param_t *)calloc(sizeof(esp_zb_zdo_bind_req_param_t), 1);
        bind_req->req_dst_addr = esp_zb_get_short_address();

        /* populate the src information of the binding */
        esp_zb_get_long_address(bind_req->src_address);
        bind_req->src_endp = BLINDS_WINDOW_COVERING_ENDPOINT_A;
        bind_req->cluster_id = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING; // ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;

        /* populate the dst information of the binding */
        bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        memcpy(bind_req->dst_address_u.addr_long, device->ieee_addr, sizeof(esp_zb_ieee_addr_t));
        bind_req->dst_endp = peer_endpoint;

        ESP_LOGI(TAG, "Bind window device");
        esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);
    }
}

/**
 * @brief Initiates discovery of window covering devices and starts binding.
 *
 * @param param Pointer to the match descriptor request parameters.
 * @param user_cb Callback to invoke when a device is found.
 * @param user_ctx Pointer to user context.
 */
static void find_window_cover_device(esp_zb_zdo_match_desc_req_param_t *param, esp_zb_zdo_match_desc_callback_t user_cb, void *user_ctx)
{
    uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING};
    param->profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    param->num_in_clusters = 1;
    param->num_out_clusters = 0;
    param->cluster_list = cluster_list;
    esp_zb_zdo_match_cluster(param, user_cb, (void *)&window_cover_device);
}

/**
 * @brief Configures attribute reporting for window covering position.
 */
static void setup_attrib_reporting(void)
{
    /* Send "read attributes" command to the window cover position */
    esp_zb_zcl_read_attr_cmd_t read_req = {0};
    read_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    read_req.zcl_basic_cmd.src_endpoint = BLINDS_WINDOW_COVERING_ENDPOINT_A;
    read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING;

    uint16_t attributes[] = {
        ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
        // ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID,
        // ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID,
        // ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_TOLERANCE_ID
    };
    read_req.attr_number = ARRAY_LENTH(attributes);
    read_req.attr_field = attributes;

    /* Send "configure report attribute" command to the bound device */
    esp_zb_zcl_config_report_cmd_t report_cmd = {0};
    report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_cmd.zcl_basic_cmd.src_endpoint = BLINDS_WINDOW_COVERING_ENDPOINT_A;
    report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING;

    int16_t report_change = 1; /* report on each 1 percentage points */
    esp_zb_zcl_config_report_record_t records[] = {
        {
            .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
            .attributeID = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
            .attrType = ESP_ZB_ZCL_ATTR_TYPE_U8, // ESP_ZB_ZCL_ATTR_TYPE_S16,
            .min_interval = 0,
            .max_interval = 0,
            .reportable_change = &report_change,
        },
    };
    report_cmd.record_number = ARRAY_LENTH(records);
    report_cmd.record_field = records;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_config_report_cmd_req(&report_cmd);
    esp_zb_lock_release();
    ESP_EARLY_LOGI(TAG, "Send 'configure reporting' command");

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_read_attr_cmd_req(&read_req);
    esp_zb_lock_release();
    ESP_EARLY_LOGI(TAG, "Send 'read attributes' command");
}

/**
 * @brief Deferred initialization of drivers and event handlers after Zigbee stack startup.
 *
 * @return ESP_OK on success, ESP_FAIL otherwise.
 */
static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;
    if (!is_inited)
    {
        // ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), esp_app_buttons_handler),
        //                     ESP_FAIL, TAG, "Failed to initialize switch driver");

        // Initialize the blinds reporting
        setup_attrib_reporting();

        // Event handler registrations
        ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_POSITION_UPDATED, &blind_position_change_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_POSITION_UPDATED");
        ESP_LOGI(TAG, "Event handlers registered.");

        is_inited = true;
    }
    return is_inited ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Zigbee application signal handler. Handles Zigbee stack events such as startup, device announce, and network events.
 *
 * @param signal_struct Pointer to the Zigbee application signal structure.
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
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
                ESP_LOGI(TAG, "Start network formation");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            }
            else
            {
                esp_zb_bdb_open_network(180);
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
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Formed network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        else
        {
            ESP_LOGI(TAG, "Restart network formation (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Network steering started");
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
        esp_zb_zdo_match_desc_req_param_t cmd_req;
        cmd_req.dst_nwk_addr = dev_annce_params->device_short_addr;
        cmd_req.addr_of_interest = dev_annce_params->device_short_addr;
        find_window_cover_device(&cmd_req, user_find_cb, NULL);
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK)
        {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p))
            {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            }
            else
            {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

/**
 * @brief Handles attribute changes for supported Zigbee clusters (Basic, Window Covering).
 *
 * @param cluster_id The cluster ID.
 * @param attribute Pointer to the attribute structure.
 */
static void esp_app_zb_attribute_handler(uint16_t cluster_id, const esp_zb_zcl_attribute_t *attribute)
{
    /* Basic cluster attributes */
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_BASIC)
    {
        if (attribute->id == ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING &&
            attribute->data.value)
        {
            const zbstring_t *zbstr = (const zbstring_t *)attribute->data.value;
            char *string = (char *)malloc(zbstr->len + 1);
            memcpy(string, zbstr->data, zbstr->len);
            string[zbstr->len] = '\0';
            ESP_LOGI(TAG, "Peer Manufacturer is \"%s\"", string);
            free(string);
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING &&
            attribute->data.value)
        {
            const zbstring_t *zbstr = (const zbstring_t *)attribute->data.value;
            char *string = (char *)malloc(zbstr->len + 1);
            memcpy(string, zbstr->data, zbstr->len);
            string[zbstr->len] = '\0';
            ESP_LOGI(TAG, "Peer Model is \"%s\"", string);
            free(string);
        }
    }

    /* Window covering cluster attributes */
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING)
    {
        if (attribute->id == ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_U8)
        {
            current_position_lift_percentage = *(uint8_t *)attribute->data.value;
            ESP_LOGI(TAG, "Current Position Lift Percentage is %d%%", current_position_lift_percentage);
        }

        else
        {
            ESP_LOGI(TAG, "Received change request to the unhandled attribute %d for the WINDOW_COVERING cluster. No change performed.", attribute->id);
        }
    }

#if 0
    /* Temperature Measurement cluster attributes */
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT)
    {
        if (attribute->id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_S16)
        {
            int16_t value = attribute->data.value ? *(int16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Measured Value is %.2f degrees Celsius", zb_s16_to_temperature(value));
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_S16)
        {
            int16_t min_value = attribute->data.value ? *(int16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Min Measured Value is %.2f degrees Celsius", zb_s16_to_temperature(min_value));
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_S16)
        {
            int16_t max_value = attribute->data.value ? *(int16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Max Measured Value is %.2f degrees Celsius", zb_s16_to_temperature(max_value));
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_TOLERANCE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
        {
            uint16_t tolerance = attribute->data.value ? *(uint16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Tolerance is %.2f degrees Celsius", 1.0 * tolerance / 100);
        }
    }
#endif
}

/**
 * @brief Handles read attribute response messages from Zigbee devices.
 *
 * @param message Pointer to the read attribute response message.
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Read attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->info.src_address.u.short_addr, message->info.src_endpoint,
             message->info.dst_endpoint, message->info.cluster);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable)
    {
        ESP_LOGI(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)",
                 variable->status, message->info.cluster,
                 variable->attribute.id, variable->attribute.data.type,
                 variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0);
        if (variable->status == ESP_ZB_ZCL_STATUS_SUCCESS)
        {
            esp_app_zb_attribute_handler(message->info.cluster, &variable->attribute);
        }

        variable = variable->next;
    }

    return ESP_OK;
}

/**
 * @brief Handles attribute reporting messages from Zigbee devices.
 *
 * @param message Pointer to the report attribute message.
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->status);
    ESP_LOGI(TAG, "Received report from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->src_address.u.short_addr, message->src_endpoint,
             message->dst_endpoint, message->cluster);
    esp_app_zb_attribute_handler(message->cluster, &message->attribute);
    return ESP_OK;
}

/**
 * @brief Handles configure reporting response messages from Zigbee devices.
 *
 * @param message Pointer to the configure report response message.
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_config_report_resp_variable_t *variable = message->variables;
    while (variable)
    {
        ESP_LOGI(TAG, "Configure report response: status(%d), cluster(0x%x), direction(0x%x), attribute(0x%x)",
                 variable->status, message->info.cluster, variable->direction, variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}

/**
 * @brief Handles window covering movement commands (open, close, stop, set position).
 *
 * @param message Pointer to the window covering movement message.
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t zb_window_covering_movement_handler(const esp_zb_zcl_window_covering_movement_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty movement message");

    ESP_LOGI(TAG, "Window covering movement command received: command=0x%02x on cluster 0x%04x",
             message->command, message->info.cluster);

    ESP_RETURN_ON_FALSE(message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                        ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    blind_id_t blind_id = BLIND_1; // TODO hardcoded for now, change to use the correct blind ID

    switch (message->command)
    {
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN:
        // Handle open/up command
        ESP_LOGI(TAG, "Received UP/OPEN command for window covering");
        app_event_post(APP_EVENT_BLIND_OPENING, &blind_id, sizeof(blind_id));
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
        // Handle close/down command
        ESP_LOGI(TAG, "Received DOWN/CLOSE command for window covering");
        app_event_post(APP_EVENT_BLIND_CLOSING, &blind_id, sizeof(blind_id));
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP:
        // Handle stop command
        ESP_LOGI(TAG, "Received STOP command for window covering");
        app_event_post(APP_EVENT_BLIND_STOPPING, &blind_id, sizeof(blind_id));
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE:
        // Handle go to lift percentage command
        ESP_LOGI(TAG, "Received GOTO LIFT PERCENTAGE command: %d%%", message->payload.percentage_lift_value);
        blind_position_t blind_position = {
            .blind_id = blind_id,
            .position = message->payload.percentage_lift_value,
        };
        app_event_post(APP_EVENT_BLIND_UPDATING_POSITION, &blind_position, sizeof(blind_position));
        break;
    default:
        ESP_LOGW(TAG, "Unknown window covering command=0x%02x", message->command);
        break;
    }

    return ESP_OK;
}

/**
 * @brief Zigbee core action handler. Dispatches Zigbee core callbacks to the appropriate handler.
 *
 * @param callback_id The callback ID.
 * @param message Pointer to the callback message.
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((const esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((const esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        ret = zb_configure_report_resp_handler((const esp_zb_zcl_cmd_config_report_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_WINDOW_COVERING_MOVEMENT_CB_ID:
        ret = zb_window_covering_movement_handler((const esp_zb_zcl_window_covering_movement_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

/**
 * @brief Creates a custom cluster list for the window covering endpoint.
 *
 * @param window_covering Pointer to the window covering configuration.
 * @return Pointer to the created cluster list.
 */
static esp_zb_cluster_list_t *custom_window_covering_clusters_create(esp_zb_window_covering_cfg_t *window_covering)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    // Basic Cluster
    esp_zb_attribute_list_t *basic_cluster_attr_list = esp_zb_basic_cluster_create(&(window_covering->basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Identify cluster
    esp_zb_attribute_list_t *identify_cluster_attr_list = esp_zb_identify_cluster_create(&(window_covering->identify_cfg));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_cluster_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

    // Covering cluster
    esp_zb_attribute_list_t *window_covering_attr_list = esp_zb_window_covering_cluster_create(&(window_covering->window_cfg));
    ESP_ERROR_CHECK(esp_zb_window_covering_cluster_add_attr(window_covering_attr_list, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, &current_position_lift_percentage));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_window_covering_cluster(cluster_list, window_covering_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    return cluster_list;
}

/**
 * @brief Creates a custom endpoint for the window covering device.
 *
 * @param endpoint_id The endpoint ID.
 * @param window_covering Pointer to the window covering configuration.
 * @return Pointer to the created endpoint list.
 */
static esp_zb_ep_list_t *custom_window_covering_ep_create(uint8_t endpoint_id, esp_zb_window_covering_cfg_t *window_covering)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_WINDOW_COVERING_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(ep_list, custom_window_covering_clusters_create(window_covering), endpoint_config);
    return ep_list;
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

    /* Create customized window cover endpoint */
    esp_zb_window_covering_cfg_t window_covering_cfg = ESP_ZB_DEFAULT_WINDOW_COVERING_CONFIG();
    esp_zb_ep_list_t *esp_zb_window_covering_ep = custom_window_covering_ep_create(BLINDS_WINDOW_COVERING_ENDPOINT_A, &window_covering_cfg);

    /* Register the device */
    esp_zb_device_register(esp_zb_window_covering_ep);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

/**
 * @brief Initializes the Zigbee stack and starts the Zigbee task.
 *
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t zigbee_init(void)
{
    // Initialize Zigbee
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    ESP_RETURN_ON_ERROR(esp_zb_platform_config(&config), TAG, "Failed to configure Zigbee platform");
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Zigbee stack initialized");

    // // Event handler registrations
    // ESP_RETURN_ON_ERROR(app_event_register(APP_EVENT_BLIND_POSITION_UPDATED, &blind_position_change_event_handler, NULL), TAG, "Failed to register event handler for APP_EVENT_BLIND_POSITION_UPDATED");
    // ESP_LOGI(TAG, "Event handlers registered.");

    return ESP_OK;
}

/**
 * @brief Handles blind position change events and updates Zigbee attribute.
 *
 * @param arg Unused.
 * @param event_base Event base (should be APP_EVENT).
 * @param event_id Event ID (should be APP_EVENT_BLIND_POSITION_UPDATED).
 * @param event_data Pointer to the blind position update data.
 */
static void blind_position_change_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_RETURN_VOID_ON_FALSE(event_base == APP_EVENT, TAG, "Received event different from APP_EVENT base by the blind_position_change_event_handler");
    ESP_RETURN_VOID_ON_FALSE(event_id == APP_EVENT_BLIND_POSITION_UPDATED, TAG, "Received event different from APP_EVENT_BLIND_POSITION_UPDATED by the blind_position_change_event_handler");

    const blind_position_t *position_update = (const blind_position_t *)event_data;

    ESP_LOGI(TAG, "Position change event received for blind %d. Position: %d%%", position_update->blind_id, position_update->position);

    current_position_lift_percentage = position_update->position; // TODO manage different values per blind_id

    esp_zb_zcl_status_t report_status = esp_zb_zcl_set_attribute_val(
        BLINDS_WINDOW_COVERING_ENDPOINT_A,
        ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
        &current_position_lift_percentage,
        false);

    if (report_status != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGE(TAG, "Failed to set attribute value: %d", report_status);
        return;
    }

#if 0
    esp_zb_zcl_report_attr_cmd_t report = {
        .zcl_basic_cmd.src_endpoint = BLINDS_WINDOW_COVERING_ENDPOINT_A,
        .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI, // verso il client (coordinator)
        .manuf_specific = 0,                          // Non è un attributo manufacturer specific
        .dis_defalut_resp = 1,                        // Disabilitiamo default response
        .manuf_code = 0x0000,                         // se non usi attributi manufacturer specific, lascia 0
        .attributeID = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID};

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report);
    esp_zb_lock_release();
#endif

    ESP_LOGI(TAG, "ZCL report successfully sent to ZHA: %d%%", current_position_lift_percentage);
}
