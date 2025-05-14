#include "mcu.h"
#include "led_strip.h"
#include "led_strip_interface.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"

#define LED_STRIP_GPIO 8
#define LED_STRIP_RMT_RES_HZ 10000000 // 10 MHz resolution
#define LED_PIN GPIO_NUM_08

#define ADC_UNIT ADC_UNIT_1 // ADC1

static const char *TAG = "mcu";

static led_strip_handle_t led_strip;
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_oneshot_chan_cfg_t chan_cfg = {
    .bitwidth = ADC_BITWIDTH_12,
    .atten = ADC_ATTEN_DB_12, // Up to 3.3V approx.
};

static SemaphoreHandle_t led_mutex = NULL;

/**
 * @brief Initialize the MCU and configure the RGB LED strip.
 *
 * This function sets up the LED strip using the RMT peripheral and clears the LED to ensure it is off.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mcu_init(void)
{
    ESP_LOGI(TAG, "Initializing MCU...");

    // Configuration for the LED strip
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,                            // GPIO pin connected to the LED strip
        .max_leds = 1,                                               // Only one RGB LED is used
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // Color format for the LED
        .led_model = LED_MODEL_WS2812,                               // LED model (e.g., WS2812)
        .flags.invert_out = false                                    // Do not invert the output signal
    };

    // Configuration for the RMT peripheral used to drive the LED strip
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,       // Default clock source for RMT
        .resolution_hz = LED_STRIP_RMT_RES_HZ // Resolution of the RMT signal
    };

    // Create a mutex for thread-safe access to the LED strip
    if (!led_mutex)
    {
        led_mutex = xSemaphoreCreateMutex();
    }

    // Create a new RMT device for the LED strip
    ESP_RETURN_ON_ERROR(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip),
                        TAG,
                        "Failed to create new RMT device");

    // Clear the LED strip to turn off all LEDs
    ESP_RETURN_ON_ERROR(led_strip_clear(led_strip), TAG, "Failed to clear the LED strip");

    // Initialize ADC
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_cfg, &adc_handle),
                        TAG,
                        "Failed to create ADC unit");

    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &chan_cfg),
                        TAG,
                        "Failed to configure ADC channel 0");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_1, &chan_cfg),
                        TAG,
                        "Failed to configure ADC channel 1");

    ESP_LOGI(TAG, "MCU initialized successfully");

    return ESP_OK;
}

/**
 * @brief Turn on the RGB LED with specified color values.
 *
 * This function sets the color of the RGB LED to the specified red, green, and blue intensity values.
 *
 * @param red Intensity of the red component (0-255).
 * @param green Intensity of the green component (0-255).
 * @param blue Intensity of the blue component (0-255).
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mcu_turn_rgb_led_on(uint32_t red, uint32_t green, uint32_t blue)
{
    xSemaphoreTake(led_mutex, portMAX_DELAY);
    // Set the color of the first LED (index 0) to the specified RGB values
    ESP_RETURN_ON_ERROR(led_strip_set_pixel(led_strip, 0, red, green, blue),
                        TAG,
                        "Failed to set pixel color");

    // Refresh the LED strip to apply the changes
    ESP_RETURN_ON_ERROR(led_strip_refresh(led_strip),
                        TAG,
                        "Failed to refresh LED strip");
    xSemaphoreGive(led_mutex);

    return ESP_OK;
}

/**
 * @brief Turn off the RGB LED.
 *
 * This function sets the color of the RGB LED to black (0, 0, 0) and refreshes the LED strip.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mcu_turn_rgb_led_off(void)
{
    ESP_RETURN_ON_ERROR(mcu_turn_rgb_led_on(0, 0, 0),
                        TAG,
                        "Failed to turn off RGB LED");
    // // Set the color of the first LED (index 0) to black (off)
    // ESP_RETURN_ON_ERROR(led_strip_set_pixel(led_strip, 0, 0, 0, 0),
    //                     TAG,
    //                     "Failed to turn off LED");

    // // Refresh the LED strip to apply the changes
    // ESP_RETURN_ON_ERROR(led_strip_refresh(led_strip),
    //                     TAG,
    //                     "Failed to refresh LED strip");

    return ESP_OK;
}

/**
 * @brief Blink the RGB LED with specified color values.
 *
 * This function blinks the RGB LED with the specified red, green, and blue intensity values for a given period and duty cycle.
 *
 * @param red Intensity of the red component (0-255).
 * @param green Intensity of the green component (0-255).
 * @param blue Intensity of the blue component (0-255).
 * @param period Duration of one blink cycle in milliseconds.
 * @param duty_cycle Duty cycle of the blink (0.0 to 1.0).
 * @param num_cycles Number of blink cycles.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mcu_blink_rgb_led(uint8_t red, uint8_t green, uint8_t blue, uint32_t period, float duty_cycle, uint8_t num_cycles)
{
    uint32_t on_time = (uint32_t)((float)period * duty_cycle + 0.5f);
    uint32_t off_time = period - on_time;

    for (uint8_t i = 0; i < num_cycles; i++)
    {
        // Set the color of the first LED (index 0) to the specified RGB values
        ESP_RETURN_ON_ERROR(mcu_turn_rgb_led_on(red, green, blue),
                            TAG,
                            "Failed to set pixel color");

        vTaskDelay(pdMS_TO_TICKS(on_time));

        // Set the color of the first LED (index 0) to black (off)
        ESP_RETURN_ON_ERROR(mcu_turn_rgb_led_off(),
                            TAG,
                            "Failed to turn off LED");

        vTaskDelay(pdMS_TO_TICKS(off_time));
    }
    return ESP_OK;
}

/**
 * @brief Get the voltage reading from a specific ADC channel.
 *
 * This function reads the ADC value from the specified channel and converts it to a voltage.
 *
 * @param voltage Pointer to store the voltage reading in millivolts.
 * @param adc_channel The ADC channel to read from.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mcu_get_adc_voltage(float *voltage, adc_channel_t adc_channel)
{
    int adc_value = 0;

    // Read the ADC value
    ESP_RETURN_ON_ERROR(adc_oneshot_read(adc_handle, adc_channel, &adc_value),
                        TAG,
                        "Failed to read ADC value from channel %d", adc_channel);

    // Convert the ADC value to a voltage (in millivolts)
    *voltage = 3.3f * (float)adc_value / 4095.0f; // Assuming 3.3V reference voltage

    return ESP_OK;
}
