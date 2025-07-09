/*  Meshtastic Compact Example Code

    *  This example demonstrates how to use the MeshtasticCompact library to receive and process
       messages, position updates, node information, and waypoints from a Meshtastic device.
    *  It initializes the radio, sets up callbacks for different message types, and prints the
       received data to the log.
    *  The code is designed to run on an ESP32S3.
*/

/*
TODO list:
 - Add has_data tags to structs
 - Add pin configuration options
 - Add frequency configuration options
 - Add channel configuration options
 - Add LONGFAST / .. options
 - Add multiple module type support
 - Add callback on ANY type of packets
 - Revise send_ack
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
    ESP_LOGW(TAG, "Source Node ID: 0x%08" PRIx32 ", Destination Node ID: 0x%08" PRIx32 ", Hop Limit: %d", header.srcnode, header.dstnode, header.hop_limit);
    ESP_LOGI(TAG, "Last signal data - RSSI: %.2f dBm, SNR: %.2f dB. WantAck: %d", header.rssi, header.snr, header.want_ack ? 1 : 0);
    auto sender = meshtasticCompact.nodeinfo_db.get(header.srcnode);
    if (sender) {
        ESP_LOGI(TAG, "Sender Node: Short Name: %s, Long Name: %s", sender->short_name, sender->long_name);
    } else {
        ESP_LOGW(TAG, "Sender Node Info not found in database for Node ID: 0x%08" PRIx32, header.srcnode);
    }
}

void app_main(void) {
    MeshtasticCompactHelpers::PositionBuilder(meshtasticCompact.my_position, 47.123456f, 18.123456f, 500, 0, 5);
    meshtasticCompact.RadioInit();

    meshtasticCompact.setOnMessage([](MC_Header& header, MC_TextMessage& message) {
        ESP_LOGI(TAG, "Received message on channel %d: %s", message.chan, message.text.c_str());
        ESP_LOGI(TAG, "Msg type %d", message.type);
        PrintHeaderInfo(header);
    });
    meshtasticCompact.setOnPositionMessage([](MC_Header& header, MC_Position& position, bool needReply) {
        ESP_LOGI(TAG, "Received position update:");
        PrintHeaderInfo(header);
        ESP_LOGI(TAG, "Latitude: %ld, Longitude: %ld, Altitude: %ld", position.latitude_i, position.longitude_i, position.altitude);
        ESP_LOGI(TAG, "Ground Speed: %lu", position.ground_speed);
        ESP_LOGI(TAG, "Satellites in view: %lu", position.sats_in_view);
        ESP_LOGI(TAG, "Location Source: %u", position.location_source);
        if (needReply && header.dstnode == meshtasticCompact.getMyNodeInfo()->node_id) {
            meshtasticCompact.SendMyPosition();  // Reply to the sender
        }
    });
    meshtasticCompact.setOnNodeInfoMessage([](MC_Header& header, MC_NodeInfo& nodeinfo, bool needReply) {
        ESP_LOGI(TAG, "Received node info:");
        PrintHeaderInfo(header);
        ESP_LOGI(TAG, "Node ID: %s", nodeinfo.id);
        ESP_LOGI(TAG, "Short Name: %s", nodeinfo.short_name);
        ESP_LOGI(TAG, "Long Name: %s", nodeinfo.long_name);
        ESP_LOGI(TAG, "Hardware Model: %u", nodeinfo.hw_model);
        ESP_LOGI(TAG, "MAC Address: %02x:%02x:%02x:%02x:%02x:%02x",
                 nodeinfo.macaddr[0], nodeinfo.macaddr[1], nodeinfo.macaddr[2],
                 nodeinfo.macaddr[3], nodeinfo.macaddr[4], nodeinfo.macaddr[5]);
        // ESP_LOGI(TAG, "Public Key: %s", nodeinfo.public_key);
        ESP_LOGI(TAG, "Role: %u", nodeinfo.role);

        if (needReply && header.dstnode == meshtasticCompact.getMyNodeInfo()->node_id) {
            meshtasticCompact.SendMyNodeInfo(header.srcnode);  // Reply to the sender
        }
    });
    meshtasticCompact.setOnWaypointMessage([](MC_Header& header, MC_Waypoint& waypoint) {
        ESP_LOGI(TAG, "Received waypoint:");
        PrintHeaderInfo(header);
        ESP_LOGI(TAG, "Waypoint ID: %lu", waypoint.id);
        ESP_LOGI(TAG, "Name: %s", waypoint.name);
        ESP_LOGI(TAG, "Description: %s", waypoint.description);
        ESP_LOGI(TAG, "Latitude: %ld, Longitude: %ld", waypoint.latitude_i, waypoint.longitude_i);
        ESP_LOGI(TAG, "Icon: %lu, Expire: %lu", waypoint.icon, waypoint.expire);
    });
    meshtasticCompact.setOnTelemetryDevice([](MC_Header& header, MC_Telemetry_Device& telemetry) {
        ESP_LOGI(TAG, "Received telemetry device data:");
        PrintHeaderInfo(header);
        ESP_LOGI(TAG, "Battery Voltage: %.2f V", telemetry.voltage);
        ESP_LOGI(TAG, "Battery Percent: %lu", telemetry.battery_level);
        ESP_LOGI(TAG, "channel_utilization: %.2f °C", telemetry.channel_utilization);
        ESP_LOGI(TAG, "Uptime: %lu seconds", telemetry.uptime_seconds);
    });
    meshtasticCompact.setOnTelemetryEnvironment([](MC_Header& header, MC_Telemetry_Environment& telemetry) {
        ESP_LOGI(TAG, "Received telemetry environment data:");
        PrintHeaderInfo(header);
        ESP_LOGI(TAG, "Temperature: %.2f °C", telemetry.temperature);
        ESP_LOGI(TAG, "Humidity: %.2f %%", telemetry.humidity);
        ESP_LOGI(TAG, "Pressure: %.2f hPa", telemetry.pressure);
        ESP_LOGI(TAG, "Lux: %.2f", telemetry.lux);
    });
    meshtasticCompact.setOnTraceroute([](MC_Header& header, MC_RouteDiscovery& route, bool for_me, bool is_reply, bool need_reply) {
        ESP_LOGI(TAG, "Received traceroute packet:");
        PrintHeaderInfo(header);
        ESP_LOGI(TAG, "Hop Limit: %d, Hop Start: %d", header.hop_limit, header.hop_start);
        ESP_LOGI(TAG, "Route Count: %zu", route.route_count);
        for (int i = 0; i < route.route_count; ++i) {
            ESP_LOGI(TAG, "Hop %d: Node ID: 0x%08" PRIx32, i + 1, route.route[i]);
        }
        ESP_LOGI(TAG, "Route BACK Count: %zu", route.route_back_count);
        for (int i = 0; i < route.route_back_count; ++i) {
            ESP_LOGI(TAG, "Hop %d: Node ID: 0x%08" PRIx32, i + 1, route.route_back[i]);
        }
        ESP_LOGI(TAG, "SNR Towards Count: %zu", route.snr_towards_count);
        for (int i = 0; i < route.snr_towards_count; ++i) {
            ESP_LOGI(TAG, "SNR Towards Hop %d: %d dB", i + 1, route.snr_towards[i]);
        }
        ESP_LOGI(TAG, "SNR Back Count: %zu", route.snr_back_count);
        for (int i = 0; i < route.snr_back_count; ++i) {
            ESP_LOGI(TAG, "SNR Back Hop %d: %d dB", i + 1, route.snr_back[i]);
        }
        ESP_LOGI(TAG, "For Me: %s, Is Reply: %s, Need Reply: %s", for_me ? "true" : "false", is_reply ? "true" : "false", need_reply ? "true" : "false");
    });

    meshtasticCompact.SendMyNodeInfo(0xffffffff, true);
    vTaskDelay(15000 / portTICK_PERIOD_MS);
    while (1) {
        meshtasticCompact.SendMyNodeInfo();
        vTaskDelay(25000 / portTICK_PERIOD_MS);
        MC_Telemetry_Environment telemetry_env;
        telemetry_env.temperature = 25.0f + (esp_random() % 100) / 10.0f;  // Random temperature between 25.0 and 26.9
        telemetry_env.humidity = 50.0f + (esp_random() % 100) / 10.0f;     // Random humidity between 50.0 and 51.9
        telemetry_env.pressure = 1013.25f + (esp_random() % 100) / 10.0f;  // Random pressure between 1013.25 and 1014.24
        telemetry_env.lux = 100.0f + (esp_random() % 100) / 10.0f;         // Random lux between 100.0 and 199.9
        meshtasticCompact.SendTelemetryEnvironment(telemetry_env);
        vTaskDelay(2500 / portTICK_PERIOD_MS);
        MC_Telemetry_Device telemetry_dev;
        telemetry_dev.voltage = 3.7f + (esp_random() % 100) / 1000.0f;              // Random voltage between 3.7 and 3.8
        telemetry_dev.battery_level = 80 + (esp_random() % 21);                     // Random battery level between 80 and 100
        telemetry_dev.uptime_seconds = 3600 + (esp_random() % 3600);                // Random uptime between 3600 and 7200 seconds
        telemetry_dev.channel_utilization = 0.5f + (esp_random() % 500) / 1000.0f;  // Random channel utilization between 0.5 and 1.0
        meshtasticCompact.SendTelemetryDevice(telemetry_dev);
        vTaskDelay(2500 / portTICK_PERIOD_MS);
    }
}
