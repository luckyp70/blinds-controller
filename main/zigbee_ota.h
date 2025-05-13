#pragma once

#include "esp_zigbee_core.h"

/**
 * @brief Zigbee OTA upgrade configuration constants.
 *
 * These macros define the manufacturer code, image type, file versions, hardware version,
 * and maximum data size for Zigbee-native OTA upgrades. Update these as needed for your product/versioning.
 */
#define OTA_CLIENT_ENDPOINT_ID 0x0C     /**< Endpoint ID for OTA upgrade client */
#define OTA_UPGRADE_MANUFACTURER 0x1002 /**< Manufacturer code for OTA image. */
#define OTA_UPGRADE_IMAGE_TYPE 0x1011   /**< Image type for OTA image. */
#define OTA_UPGRADE_MAX_DATA_SIZE 223   /* Maximum data size for OTA upgrade */

/* Software version defaulted to v1.0.0.0 */
#ifndef OTA_UPGRADE_RUNNING_FILE_VERSION
#define OTA_UPGRADE_RUNNING_FILE_VERSION 0x01000000
#endif

/* Hardware version defaulted to v1.0 */
#ifndef OTA_UPGRADE_HW_VERSION
#define OTA_UPGRADE_HW_VERSION 0x0100
#endif

#define OTA_ELEMENT_HEADER_LEN 6 /**< OTA element format header size (tag identifier + length field). */

/**
 * @brief Tag identifier for OTA element data type.
 *
 * This enum denotes the type and format of the data within the OTA element.
 */
typedef enum esp_ota_element_tag_id_e
{
    UPGRADE_IMAGE = 0x0000, /*!< Upgrade image */
} esp_ota_element_tag_id_t;

/**
 * @brief Initialize the downloaded OTA file version from NVS at startup.
 *
 * This should be called during system initialization before using the downloaded file version.
 */
esp_err_t zb_ota_init_downloaded_file_version(void);

/**
 * @brief Get the currently stored downloaded OTA file version (persisted in NVS).
 *
 * @return The downloaded OTA file version as a 32-bit integer.
 */
uint32_t zb_ota_get_downloaded_file_version(void);

/**
 * @brief Set and persist the downloaded OTA file version in NVS.
 *
 * @param version The new downloaded OTA file version to store.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t zb_ota_set_downloaded_file_version(uint32_t version);

/**
 * @brief Add the Zigbee OTA Upgrade cluster to the cluster list.
 *
 * @param ota_cfg Pointer to the OTA cluster configuration structure.
 * @param ep_list Pointer to the endpoint list to add the OTA cluster to.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t zb_ota_ep_add(esp_zb_ota_cluster_cfg_t *ota_cfg, esp_zb_ep_list_t *ep_list);

/**
 * @brief Handle OTA upgrade status messages from the Zigbee OTA cluster.
 *
 * @param message Pointer to the OTA upgrade value message.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t zb_ota_upgrade_status_handler(const esp_zb_zcl_ota_upgrade_value_message_t *message);

/**
 * @brief Handle OTA image query response from the OTA server.
 *
 * @param message Pointer to the OTA upgrade query image response message.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t zb_ota_upgrade_query_image_resp_handler(const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *message);
