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

#include "ra01s.h"
#include "meshtastic/mesh.pb.h"
#include "pb.h"
#include "pb_decode.h"

static const char* TAG = "wifi softAP";

bool pb_decode_from_bytes(const uint8_t* srcbuf, size_t srcbufsize, const pb_msgdesc_t* fields, void* dest_struct) {
    pb_istream_t stream = pb_istream_from_buffer(srcbuf, srcbufsize);
    if (!pb_decode(&stream, fields, &dest_struct)) {
        ESP_LOGI("PB", "Can't decode protobuf reason='%s', pb_msgdesc %p", PB_GET_ERROR(&stream), fields);
        return false;
    } else {
        return true;
    }
}

void task_secondary(void* pvParameters) {
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    uint8_t txData[256];  // Maximum Payload size of SX1261/62/68 is 255
    uint8_t rxData[256];  // Maximum Payload size of SX1261/62/68 is 255
    while (1) {
        uint8_t rxLen = LoRaReceive(rxData, sizeof(rxData));
        if (rxLen > 0) {
            printf("Receive rxLen:%d\n", rxLen);
            for (int i = 0; i < rxLen; i++) {
                printf("%02x ", rxData[i]);
            }
            printf("\n");

            for (int i = 0; i < rxLen; i++) {
                if (rxData[i] > 0x19 && rxData[i] < 0x7F) {
                    char myChar = rxData[i];
                    printf("%c", myChar);
                } else {
                    printf("?");
                }
            }
            printf("\n");

            int8_t rssi, snr;
            GetPacketStatus(&rssi, &snr);
            printf("rssi=%d[dBm] snr=%d[dB]\n", rssi, snr);
            meshtastic_MeshPacket mesh_packet = meshtastic_MeshPacket_init_zero;
            bool ret = pb_decode_from_bytes(rxData, rxLen, meshtastic_MeshPacket_fields, &mesh_packet);
            if (ret) {
                ESP_LOGI(TAG, "Decoded MeshPacket: from=%" PRIu32 ", to=%" PRIu32 ", channel=%" PRIu8 ", id=%" PRIu32,
                         mesh_packet.from, mesh_packet.to, mesh_packet.channel, mesh_packet.id);
                ESP_LOGI(TAG, "rx_time=%" PRIu32 ", rx_snr=%.2f, hop_limit=%" PRIu8,
                         mesh_packet.rx_time, mesh_packet.rx_snr, mesh_packet.hop_limit);
            } else {
                ESP_LOGE(TAG, "Failed to decode MeshPacket");
            }
        }
        vTaskDelay(1);  // Avoid WatchDog alerts
    }  // end while

    // never reach here
    vTaskDelete(NULL);
}

void app_main(void) {
    ESP_LOGI(TAG, "Init");
    LoRaInit();
    int8_t txPowerInDbm = 21;

    uint32_t frequencyInHz = 0;
    frequencyInHz = 433125000;
    float tcxoVoltage = 3.3;      // don't use TCXO
    bool useRegulatorLDO = true;  // use only LDO in all modes
    if (LoRaBegin(frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) != 0) {
        ESP_LOGE(TAG, "Does not recognize the module");
        while (1) {
            vTaskDelay(1);
        }
    }
    uint8_t spreadingFactor = 11;
    uint8_t bandwidth = 2;  // 250
    uint8_t codingRate = 5;
    uint16_t preambleLength = 8;

    uint8_t payloadLen = 0;
    bool crcOn = true;
    bool invertIrq = false;
    LoRaConfig(spreadingFactor, bandwidth, codingRate, preambleLength, payloadLen, crcOn, invertIrq);
    xTaskCreate(&task_secondary, "SECONDARY", 1024 * 4, NULL, 5, NULL);
}
