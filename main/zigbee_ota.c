#include "zigbee_ota.h"

#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "nvs.h"

#define OTA_NVS_NAMESPACE "zigbee_ota"
#define OTA_NVS_KEY "dl_file_ver"

static const char *TAG = "zigbee_ota";

static const esp_partition_t *s_ota_partition = NULL;
static esp_ota_handle_t s_ota_handle = 0;
static bool s_tagid_received = false;
static uint32_t s_downloaded_file_version = 0x01000000; // Default to v1.0.0.0
static esp_zb_zcl_ota_upgrade_client_variable_t variable_config = {
    .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
    .hw_version = OTA_UPGRADE_HW_VERSION,
    .max_data_size = OTA_UPGRADE_MAX_DATA_SIZE,
};
static uint16_t ota_upgrade_server_addr = 0xffff;
static uint8_t ota_upgrade_server_ep = 0xff;

static esp_err_t esp_element_ota_data(uint32_t total_size, const void *payload, uint16_t payload_size, void **outbuf, uint16_t *outlen);

/**
 * @brief Initialize the downloaded OTA file version from NVS at startup.
 *
 * This should be called during system initialization before using the downloaded file version.
 */
esp_err_t zb_ota_init_downloaded_file_version(void)
{
    esp_err_t ret = ESP_OK;
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(OTA_NVS_NAMESPACE, NVS_READONLY, &nvs), TAG, "Failed to open NVS namespace");

    ESP_GOTO_ON_ERROR(nvs_get_u32(nvs, OTA_NVS_KEY, &s_downloaded_file_version), finally, TAG, "Failed to get downloaded file version");

finally:
    nvs_close(nvs);
    return ret;
}

/**
 * @brief Get the currently stored downloaded OTA file version (persisted in NVS).
 *
 * @return The downloaded OTA file version as a 32-bit integer.
 */
uint32_t zb_ota_get_downloaded_file_version(void)
{
    return s_downloaded_file_version;
}

/**
 * @brief Set and persist the downloaded OTA file version in NVS.
 *
 * @param version The new downloaded OTA file version to store.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t zb_ota_set_downloaded_file_version(uint32_t version)
{
    esp_err_t ret = ESP_OK;
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(OTA_NVS_NAMESPACE, NVS_READWRITE, &nvs), TAG, "Failed to open NVS namespace");

    ESP_GOTO_ON_ERROR(nvs_set_u32(nvs, OTA_NVS_KEY, version), finally, TAG, "Failed to set downloaded file version");
    ESP_GOTO_ON_ERROR(nvs_commit(nvs), finally, TAG, "Failed to commit downloaded file version");

    s_downloaded_file_version = version;

finally:
    nvs_close(nvs);
    return ret;
}

/**
 * @brief Add the Zigbee OTA Upgrade cluster to the cluster list.
 *
 * @param cluster_list Pointer to the cluster list to add the OTA cluster to.
 * @param ota_cfg Pointer to the OTA cluster configuration structure.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t zb_ota_ep_add(esp_zb_ota_cluster_cfg_t *ota_cfg, esp_zb_ep_list_t *ep_list)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = OTA_CLIENT_ENDPOINT_ID,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEST_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_attribute_list_t *ota_cluster_attr_list = esp_zb_ota_cluster_create(ota_cfg);
    ESP_RETURN_ON_ERROR(esp_zb_ota_cluster_add_attr(ota_cluster_attr_list, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, (void *)&variable_config), TAG, "Failed to add OTA upgrade client data attribute");
    ESP_RETURN_ON_ERROR(esp_zb_ota_cluster_add_attr(ota_cluster_attr_list, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, (void *)&ota_upgrade_server_addr), TAG, "Failed to add OTA upgrade server address attribute");
    ESP_RETURN_ON_ERROR(esp_zb_ota_cluster_add_attr(ota_cluster_attr_list, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, (void *)&ota_upgrade_server_ep), TAG, "Failed to add OTA upgrade server endpoint attribute");
    ESP_RETURN_ON_ERROR(esp_zb_cluster_list_add_ota_cluster(cluster_list, ota_cluster_attr_list, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE), TAG, "Failed to add OTA cluster to cluster list");

    ESP_RETURN_ON_ERROR(esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config), TAG, "Failed to add OTA endpoint to list");
    return ESP_OK;
}

/**
 * @brief Handle OTA upgrade status messages from the Zigbee OTA cluster.
 *
 * This function processes OTA status events (start, receive, apply, check, finish) and manages the OTA partition and flashing process.
 *
 * @param message Pointer to the OTA upgrade value message.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t zb_ota_upgrade_status_handler(const esp_zb_zcl_ota_upgrade_value_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");

    static uint32_t total_size = 0;
    static uint32_t offset = 0;
    static int64_t start_time = 0;
    esp_err_t ret = ESP_OK;

    if (message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        switch (message->upgrade_status)
        {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
            ESP_LOGI(TAG, "-- OTA upgrade start");
            start_time = esp_timer_get_time();
            s_ota_partition = esp_ota_get_next_update_partition(NULL);
            assert(s_ota_partition);

            ret = esp_ota_begin(s_ota_partition, 0, &s_ota_handle);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to begin OTA partition, status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
            total_size = message->ota_header.image_size;
            offset += message->payload_size;
            ESP_LOGI(TAG, "-- OTA Client receives data: progress [%ld/%ld]", offset, total_size);
            if (message->payload_size && message->payload)
            {
                uint16_t payload_size = 0;
                void *payload = NULL;
                ret = esp_element_ota_data(total_size, message->payload, message->payload_size, &payload, &payload_size);
                ESP_RETURN_ON_ERROR(ret, TAG, "Failed to element OTA data, status: %s", esp_err_to_name(ret));

                ret = esp_ota_write(s_ota_handle, (const void *)payload, payload_size);
                ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write OTA data to partition, status: %s", esp_err_to_name(ret));
            }
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
            ESP_LOGI(TAG, "-- OTA upgrade apply");
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            ret = offset == total_size ? ESP_OK : ESP_FAIL;
            offset = 0;
            total_size = 0;
            s_tagid_received = false;
            ESP_LOGI(TAG, "-- OTA upgrade check status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
            ESP_LOGI(TAG, "-- OTA Finish");
            ESP_LOGI(TAG, "-- OTA Information: version: 0x%lx, manufacturer code: 0x%x, image type: 0x%x, total size: %ld bytes, cost time: %lld ms,",
                     message->ota_header.file_version, message->ota_header.manufacturer_code, message->ota_header.image_type,
                     message->ota_header.image_size, (esp_timer_get_time() - start_time) / 1000);

            // Save the new downloaded file version to NVS before reboot
            zb_ota_set_downloaded_file_version(message->ota_header.file_version);

            ret = esp_ota_end(s_ota_handle);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to end OTA partition, status: %s", esp_err_to_name(ret));
            ret = esp_ota_set_boot_partition(s_ota_partition);
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set OTA boot partition, status: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "Prepare to restart system");
            esp_restart();
            break;
        default:
            ESP_LOGI(TAG, "OTA status: %d", message->upgrade_status);
            break;
        }
    }
    return ret;
}

/**
 * @brief Handle OTA image query response from the OTA server.
 *
 * This function logs the OTA image information and determines whether to approve or reject the upgrade.
 *
 * @param message Pointer to the OTA upgrade query image response message.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t zb_ota_upgrade_query_image_resp_handler(const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");

    esp_err_t ret = ESP_OK;
    if (message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        // Update OTA server address and endpoint with actual values from the response
        ota_upgrade_server_addr = message->server_addr.u.short_addr;
        ota_upgrade_server_ep = message->server_endpoint;

        ESP_LOGI(TAG, "Queried OTA image from address: 0x%04hx, endpoint: %d", ota_upgrade_server_addr, ota_upgrade_server_ep);
        ESP_LOGI(TAG, "Image version: 0x%lx, manufacturer code: 0x%x, image size: %ld", message->file_version, message->manufacturer_code,
                 message->image_size);
    }
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Approving OTA image upgrade");
    }
    else
    {
        ESP_LOGI(TAG, "Rejecting OTA image upgrade, status: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Extract OTA image data from the Zigbee OTA element payload.
 *
 * This function parses the OTA element header and returns a pointer to the image data buffer and its length.
 *
 * @param total_size Total size of the OTA element.
 * @param payload Pointer to the OTA element payload.
 * @param payload_size Size of the payload in bytes.
 * @param outbuf Output pointer to the data buffer.
 * @param outlen Output pointer to the data length.
 * @return ESP_OK on success, error code otherwise.
 */
static esp_err_t esp_element_ota_data(uint32_t total_size, const void *payload, uint16_t payload_size, void **outbuf, uint16_t *outlen)
{
    static uint16_t tagid = 0;
    void *data_buf = NULL;
    uint16_t data_len;

    // Parse OTA element header if not already received
    if (!s_tagid_received)
    {
        uint32_t length = 0;
        if (!payload || payload_size <= OTA_ELEMENT_HEADER_LEN)
        {
            ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "Invalid element format");
        }

        tagid = *(const uint16_t *)payload;
        length = *(const uint32_t *)(payload + sizeof(tagid));
        if ((length + OTA_ELEMENT_HEADER_LEN) != total_size)
        {
            ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "Invalid element length [%ld/%ld]", length, total_size);
        }

        s_tagid_received = true;

        data_buf = (void *)(payload + OTA_ELEMENT_HEADER_LEN);
        data_len = payload_size - OTA_ELEMENT_HEADER_LEN;
    }
    else
    {
        data_buf = (void *)payload;
        data_len = payload_size;
    }

    // Return pointer to image data for UPGRADE_IMAGE tag
    switch (tagid)
    {
    case UPGRADE_IMAGE:
        *outbuf = data_buf;
        *outlen = data_len;
        break;
    default:
        ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "Unsupported element tag identifier %d", tagid);
        break;
    }

    return ESP_OK;
}
