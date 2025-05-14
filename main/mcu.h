#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

/**
 * @brief Initialize the MCU and configure the RGB LED strip.
 *
 * This function sets up the LED strip using the RMT peripheral and clears the LED to ensure it is off.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mcu_init(void);

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
esp_err_t mcu_turn_rgb_led_on(uint32_t red, uint32_t green, uint32_t blue);

/**
 * @brief Turn off the RGB LED.
 *
 * This function sets the color of the RGB LED to black (0, 0, 0) and refreshes the LED strip.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mcu_turn_rgb_led_off(void);

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
esp_err_t mcu_blink_rgb_led(uint8_t red, uint8_t green, uint8_t blue, uint32_t period, float duty_cycle, uint8_t num_cycles);

/**
 * @brief Get the voltage reading from a specific ADC channel.
 *
 * This function reads the ADC value from the specified channel and converts it to a voltage.
 *
 * @param adc_channel The ADC channel to read from.
 *
 * @return The voltage reading in millivolts, or -1 on error.
 */
esp_err_t mcu_get_adc_voltage(float *voltage, adc_channel_t adc_channel);