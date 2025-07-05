/*  Meshtastic Compact Example Code

    *  This example demonstrates how to use the MeshtasticCompact library to receive and process
       messages, position updates, node information, and waypoints from a Meshtastic device.
    *  It initializes the radio, sets up callbacks for different message types, and prints the
       received data to the log.
    *  The code is designed to run on an ESP32S3.
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
#include "esp_random.h"
#include "MeshtasticCompact.hpp"

static const char* TAG = "MeshCompExample";
MeshtasticCompact meshtasticCompact;

extern "C" {
void app_main();
}

void PrintHeaderInfo(MC_Header& header) {
    ESP_LOGI(TAG, "Source Node ID: 0x%08" PRIx32 ", Destination Node ID: 0x%08" PRIx32 ", Hop Limit: %d", header.srcnode, header.dstnode, header.hop_limit);
    ESP_LOGI(TAG, "Last signal data - RSSI: %.2f dBm, SNR: %.2f dB. WantAck: %d", header.rssi, header.snr, header.want_ack ? 1 : 0);
    auto sender = meshtasticCompact.nodeinfo_db.get(header.srcnode);
    if (sender) {
        ESP_LOGI(TAG, "Sender Node: Short Name: %s, Long Name: %s", sender->short_name, sender->long_name);
    } else {
        ESP_LOGW(TAG, "Sender Node Info not found in database for Node ID: 0x%08" PRIx32, header.srcnode);
    }
}

void app_main(void) {
    meshtasticCompact.RadioInit();
    meshtasticCompact.setOnMessage([](MC_Header header, MC_TextMessage& message) {
        ESP_LOGI(TAG, "Received message on channel %d: %s", message.chan, message.text.c_str());
        ESP_LOGI(TAG, "Msg type %d", message.type);
        PrintHeaderInfo(header);
    });
    meshtasticCompact.setOnPositionMessage([](MC_Header header, MC_Position& position) {
        ESP_LOGI(TAG, "Received position update:");
        ESP_LOGI(TAG, "Latitude: %ld, Longitude: %ld, Altitude: %ld", position.latitude_i, position.longitude_i, position.altitude);
        ESP_LOGI(TAG, "Ground Speed: %lu", position.ground_speed);
        ESP_LOGI(TAG, "Satellites in view: %lu", position.sats_in_view);
        ESP_LOGI(TAG, "Location Source: %u", position.location_source);
        PrintHeaderInfo(header);
    });
    meshtasticCompact.setOnNodeInfoMessage([](MC_Header header, MC_NodeInfo& nodeinfo) {
        ESP_LOGI(TAG, "Received node info:");
        ESP_LOGI(TAG, "Node ID: %s", nodeinfo.id);
        ESP_LOGI(TAG, "Short Name: %s", nodeinfo.short_name);
        ESP_LOGI(TAG, "Long Name: %s", nodeinfo.long_name);
        ESP_LOGI(TAG, "Hardware Model: %u", nodeinfo.hw_model);
        ESP_LOGI(TAG, "MAC Address: %02x:%02x:%02x:%02x:%02x:%02x",
                 nodeinfo.macaddr[0], nodeinfo.macaddr[1], nodeinfo.macaddr[2],
                 nodeinfo.macaddr[3], nodeinfo.macaddr[4], nodeinfo.macaddr[5]);
        // ESP_LOGI(TAG, "Public Key: %s", nodeinfo.public_key);
        ESP_LOGI(TAG, "Role: %u", nodeinfo.role);
        PrintHeaderInfo(header);
    });
    meshtasticCompact.setOnWaypointMessage([](MC_Header header, MC_Waypoint& waypoint) {
        ESP_LOGI(TAG, "Received waypoint:");
        ESP_LOGI(TAG, "Waypoint ID: %lu", waypoint.id);
        ESP_LOGI(TAG, "Name: %s", waypoint.name);
        ESP_LOGI(TAG, "Description: %s", waypoint.description);
        ESP_LOGI(TAG, "Latitude: %ld, Longitude: %ld", waypoint.latitude_i, waypoint.longitude_i);
        ESP_LOGI(TAG, "Icon: %lu, Expire: %lu", waypoint.icon, waypoint.expire);
        PrintHeaderInfo(header);
    });
    meshtasticCompact.SendMyNodeInfo(0xffffffff, true);
    vTaskDelay(15000 / portTICK_PERIOD_MS);
    MC_Position position;
    while (1) {
        meshtasticCompact.SendMyNodeInfo();
        vTaskDelay(25000 / portTICK_PERIOD_MS);
    }
}
