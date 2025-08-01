/*  Meshtastic Compact Example Code

    *  This example demonstrates how to use the MeshtasticCompact library to receive and process
       messages, position updates, node information, and waypoints from a Meshtastic device.
    *  It initializes the radio, sets up callbacks for different message types, and prints the
       received data to the log.
    *  The code is designed to run on an ESP32S3.
*/

/*
TODO list:
 - Add multiple module type support
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

extern "C" {
void app_main();
}

Radio_PINS radio_pins = {9, 11, 10, 8, 14, 12, 13};  // Default radio pins for Heltec WSL V3.
LoraConfig lora_config = {
    .frequency = 433.125,  // default frequency
    .bandwidth = 250.0,
    .spreading_factor = 11,
    .coding_rate = 5,
    .sync_word = 0x2b,
    .preamble_length = 16,
    .output_power = 22,
    .tcxo_voltage = 1.8,
    .use_regulator_ldo = false,
};  // default LoRa configuration for EU LONGFAST 433
MeshtasticCompact meshtasticCompact;

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
    meshtasticCompact.RadioInit(RadioType::SX1262, radio_pins, lora_config);  // Initialize the radio with the specified pins and configuration
    ESP_LOGI(TAG, "Radio initialized successfully");
    meshtasticCompact.setOnMessage([](MC_Header& header, MC_TextMessage& message) {
        ESP_LOGI(TAG, "Received message on channel %d: %s", message.chan, message.text.c_str());
        ESP_LOGI(TAG, "Msg type %d", message.type);
        PrintHeaderInfo(header);
    });
    meshtasticCompact.setOnPositionMessage([](MC_Header& header, MC_Position& position, bool needReply) {
        ESP_LOGI(TAG, "Received position update:");
        PrintHeaderInfo(header);
        if (position.has_latitude_i && position.has_longitude_i) ESP_LOGI(TAG, "Latitude: %ld, Longitude: %ld, Altitude: %ld", position.latitude_i, position.longitude_i, position.altitude);
        if (position.has_ground_speed) ESP_LOGI(TAG, "Ground Speed: %lu", position.ground_speed);
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
        ESP_LOGI(TAG, "channel_utilization: %.2f ", telemetry.channel_utilization);
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
    ESP_LOGI(TAG, "Setup done");
    meshtasticCompact.SendMyNodeInfo(0xffffffff, true);
    while (1) {
        meshtasticCompact.SendMyNodeInfo();
        vTaskDelay(pdMS_TO_TICKS(90000));  // Send node info every 90 seconds
    }
}
