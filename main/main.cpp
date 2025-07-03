/*  WiFi softAP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "MeshtasticCompact.hpp"

static const char* TAG = "MeshCompExample";
MeshtasticCompact meshtasticCompact;

extern "C" {
void app_main();
}

void app_main(void) {
    meshtasticCompact.RadioInit();
    meshtasticCompact.RadioListen();
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
