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
    meshtasticCompact.setOnMessage([](MC_Header header, MC_TextMessage message) {
        ESP_LOGI(TAG, "Received message on channel %d: %s", message.chan, message.text.c_str());
        ESP_LOGI(TAG, "Msg type %d", message.type);
        ESP_LOGI(TAG, "Source Node ID: 0x%08" PRIx32 ", Destination Node ID: 0x%08" PRIx32 ", Hop Limit: %d", header.srcnode, header.dstnode, header.hop_limit);
        ESP_LOGI(TAG, "Last signal data - RSSI: %.2f dBm, SNR: %.2f dB", header.rssi, header.snr);
    });
    meshtasticCompact.setOnPositionMessage([](MC_Header header, MC_Position position) {
        ESP_LOGI(TAG, "Received position update:");
        ESP_LOGI(TAG, "Latitude: %ld, Longitude: %ld, Altitude: %ld", position.latitude_i, position.longitude_i, position.altitude);
        ESP_LOGI(TAG, "Ground Speed: %lu", position.ground_speed);
        ESP_LOGI(TAG, "Satellites in view: %lu", position.sats_in_view);
        ESP_LOGI(TAG, "Location Source: %u", position.location_source);
        ESP_LOGI(TAG, "Source Node ID: 0x%08" PRIx32 ", Destination Node ID: 0x%08" PRIx32 ", Hop Limit: %d", header.srcnode, header.dstnode, header.hop_limit);
        ESP_LOGI(TAG, "Last signal data - RSSI: %.2f dBm, SNR: %.2f dB", header.rssi, header.snr);
    });
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
