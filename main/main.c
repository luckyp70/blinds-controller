/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Fortunato Pasqualone.
 *
 * This file is part of the ESP32 Blinds Controller project.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.

     __               __
    / /   __  _______/ /____  ______
   / /   / / / / ___/ //_/ / / / __ \
  / /___/ /_/ / /__/ ,< / /_/ / /_/ /
 /_____/\__,_/\___/_/|_|\__, / .___/
                      /____/_/

*/

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"

#include "nvs_flash.h"
#include "esp_log.h"

#include "app_events.h"
#include "motors.h"
#include "buttons.h"
#include "zigbee.h"
#include "blinds.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32 Blinds Controller...");

    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize modules
    ESP_ERROR_CHECK(app_event_init());

    motors_init();
    buttons_init();
    zigbee_init();
    blinds_init();

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
