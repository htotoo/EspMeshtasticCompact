
#include "MeshtasticCompact.hpp"
#include "esp_log.h"
#include "meshtastic/mesh.pb.h"
#include "pb.h"
#include "pb_decode.h"
#include "mbedtls/aes.h"
#include <arpa/inet.h>

#define TAG "MeshtasticCompact"

#define PACKET_FLAGS_HOP_LIMIT_MASK 0x07
#define PACKET_FLAGS_WANT_ACK_MASK 0x08
#define PACKET_FLAGS_VIA_MQTT_MASK 0x10
#define PACKET_FLAGS_HOP_START_MASK 0xE0
#define PACKET_FLAGS_HOP_START_SHIFT 5

volatile bool packetFlag = false;

void IRAM_ATTR onPacketReceived() {
    packetFlag = true;
}
MeshtasticCompact::MeshtasticCompact() {}

MeshtasticCompact::~MeshtasticCompact() {}
bool MeshtasticCompact::RadioInit() {
    ESP_LOGI(TAG, "Init");
    int state = radio.begin(433.125, 250.0, 11, 5, 0x2b, 10, 16, 1.8, false);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "failed, code %d\n", state);
        while (true) {
            hal->delay(1000);
        }
    }
    ESP_LOGI(TAG, "success!\n");
    state |= radio.setCurrentLimit(130.0);
    state |= radio.explicitHeader();
    state |= radio.setCRC(RADIOLIB_SX126X_LORA_CRC_ON);
    state |= radio.setDio2AsRfSwitch(false);
    radio.setDio1Action(onPacketReceived);
    state |= radio.setRxBoostedGainMode(true);
    state |= radio.startReceive();
    if (state != 0) {
        ESP_LOGI(TAG, "Radio init failed, code %d\n", state);
    }
    return true;
}

void MeshtasticCompact::task_listen(void* pvParameters) {
    MeshtasticCompact* mshcomp = static_cast<MeshtasticCompact*>(pvParameters);
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    uint8_t rxData[256];  // Maximum Payload size of SX1261/62/68 is 255
    // mshcomp->radio.startReceive();
    while (1) {
        if (packetFlag) {
            packetFlag = false;
            printf("packetFlag!\n");
            int rxLen = mshcomp->radio.getPacketLength();
            if (rxLen > 255) rxLen = 255;  // Ensure we do not overflow the buffer
            int err = mshcomp->radio.readData(rxData, rxLen);
            // uint8_t rxLen = LoRaReceive(rxData, sizeof(rxData));
            mshcomp->radio.startReceive();
            if (err >= 0) {
                float rssi, snr;
                rssi = mshcomp->radio.getRSSI();
                snr = mshcomp->radio.getSNR();
                printf("rssi=%f[dBm] snr=%f[dB]\n", rssi, snr);
                mshcomp->DebugPacket(rxData, rxLen);
            }
            if (err < 0) {
                if (err == RADIOLIB_ERR_RX_TIMEOUT) {
                    // timeout occurred while waiting for a packet
                    printf("timeout!\n");
                } else if (err == RADIOLIB_ERR_CRC_MISMATCH) {
                    // packet was received, but is malformed
                    printf("CRC error!\n");
                } else {
                    // some other error occurred
                    printf("failed, code %d", err);
                }
            }
        }

        vTaskDelay(1);  // Avoid WatchDog alerts
    }  // end while

    // never reach here
    vTaskDelete(NULL);
}

bool MeshtasticCompact::RadioListen() {
    xTaskCreate(&task_listen, "RadioListen", 1024 * 4, this, 5, NULL);
    return true;
}

bool MeshtasticCompact::pb_decode_from_bytes(const uint8_t* srcbuf, size_t srcbufsize, const pb_msgdesc_t* fields, void* dest_struct) {
    pb_istream_t stream = pb_istream_from_buffer(srcbuf, srcbufsize);
    if (!pb_decode(&stream, fields, dest_struct)) {
        ESP_LOGI("PB", "Can't decode protobuf reason='%s', pb_msgdesc %p", PB_GET_ERROR(&stream), fields);
        return false;
    } else {
        return true;
    }
}

bool MeshtasticCompact::DebugPacket(uint8_t* data, int len) {
    if (len > 0) {
        printf("Receive rxLen:%d\n", len);
        for (int i = 0; i < len; i++) {
            printf("%02x ", data[i]);
        }
        printf("\n");

        for (int i = 0; i < len; i++) {
            if (data[i] > 0x19 && data[i] < 0x7F) {
                char myChar = data[i];
                printf("%c", myChar);
            } else {
                printf("?");
            }
        }
        printf("\n");

        // https://meshtastic.org/docs/overview/mesh-algo/#layer-1-unreliable-zero-hop-messaging
        if (len < 0x10) {
            ESP_LOGE(TAG, "Received packet too short: %d bytes", len);
            return false;
        }
        uint32_t packet_dest_net, packet_src_net, packet_id_net;

        // Copy the bytes directly from the buffer
        memcpy(&packet_dest_net, &data[0], sizeof(uint32_t));
        memcpy(&packet_src_net, &data[4], sizeof(uint32_t));
        memcpy(&packet_id_net, &data[8], sizeof(uint32_t));

        // Convert from Network-to-Host-Long byte order
        uint32_t packet_dest = (packet_dest_net);
        uint32_t packet_src = (packet_src_net);
        uint32_t packet_id = (packet_id_net);

        uint8_t packet_flags = data[12];
        uint8_t packet_chan_hash = data[13];
        uint8_t packet_next_hop = data[14];
        uint8_t packet_relay_node = data[15];
        ESP_LOGI(TAG, "Received packet: dest=0x%08lX, src=0x%08lX, id=%" PRIu32 ", flags=0x%02X, chan_hash=0x%02X, next_hop=%d, relay_node=%d",
                 packet_dest, packet_src, packet_id, packet_flags, packet_chan_hash, packet_next_hop, packet_relay_node);
        // extract flags  https://github.com/meshtastic/firmware/blob/e505ec847e20167ceca273fe22872720a5df7439/src/mesh/RadioLibInterface.cpp#L453
        uint8_t packet_hop_limit = packet_flags & PACKET_FLAGS_HOP_LIMIT_MASK;
        bool packet_want_Ack = !!(packet_flags & PACKET_FLAGS_WANT_ACK_MASK);
        bool packet_mqtt = !!(packet_flags & PACKET_FLAGS_VIA_MQTT_MASK);
        uint8_t packet_hop_start = (packet_flags & PACKET_FLAGS_HOP_START_MASK) >> PACKET_FLAGS_HOP_START_SHIFT;
        ESP_LOGI(TAG, "Header flags: hop_limit=%u, want_ack=%d, mqtt=%d, hopstart=%u", packet_hop_limit, packet_want_Ack, packet_mqtt, packet_hop_start);

        uint8_t decrypted_data[256] = {0};
        ESP_LOGI(TAG, "Attempting to decrypt payload...");
        // Zero out the remaining bytes in data[] after the actual payload
        if (len < sizeof(data)) {
            memset(&data[len], 0, sizeof(data) - len);
        }
        if (decrypt_meshtastic_payload(default_channel_key, packet_id, packet_src, &data[16], decrypted_data, len - 16)) {
            ESP_LOGI(TAG, "Decryption successful!");
            ESP_LOG_BUFFER_HEX(TAG, decrypted_data, len - 16);
            meshtastic_MeshPacket mesh_packet = meshtastic_MeshPacket_init_zero;
            bool ret = pb_decode_from_bytes(decrypted_data, len - 16, meshtastic_MeshPacket_fields, &mesh_packet);
            if (ret) {
                ESP_LOGI(TAG, "Decoded MeshPacket: from=%" PRIu32 ", to=%" PRIu32 ", channel=%" PRIu8 ", id=%" PRIu32,
                         mesh_packet.from, mesh_packet.to, mesh_packet.channel, mesh_packet.id);
                ESP_LOGI(TAG, "rx_time=%" PRIu32 ", rx_snr=%.2f, hop_limit=%" PRIu8,
                         mesh_packet.rx_time, mesh_packet.rx_snr, mesh_packet.hop_limit);
            } else {
                ESP_LOGE(TAG, "Failed to decode MeshPacket");
            }
            return ret;
        } else {
            ESP_LOGE(TAG, "Decryption failed!");
            return false;
        }
    }
    return false;
}

void MeshtasticCompact::generate_meshtastic_nonce(uint32_t packet_id, uint32_t from_node, uint8_t* nonce) {
    // Zero out the nonce buffer initially
    memset(nonce, 0, 16);

    // Bytes 0-3: Little-endian packet ID
    memcpy(nonce, &packet_id, sizeof(uint32_t));

    // Bytes 8-11: Little-endian sender node ID
    memcpy(nonce + 8, &from_node, sizeof(uint32_t));
}

/**
 * @brief Decrypts a Meshtastic payload using AES-256-CTR.
 *
 * @param key           A pointer to the 32-byte AES channel key.
 * @param packet_id     The packet ID from the cleartext header.
 * @param from_node     The sender node ID from the cleartext header.
 * @param encrypted_in  A pointer to the buffer containing the encrypted data.
 * @param decrypted_out A pointer to the buffer where the decrypted data will be written.
 * This buffer must be at least as large as the input buffer.
 * @param len           The length of the data to decrypt.
 * @return              True on success, false on failure.
 */
bool MeshtasticCompact::decrypt_meshtastic_payload(const uint8_t* key, uint32_t packet_id, uint32_t from_node, const uint8_t* encrypted_in, uint8_t* decrypted_out, size_t len) {
    // 1. Initialize the mbedtls AES context
    mbedtls_aes_context aes_ctx;
    mbedtls_aes_init(&aes_ctx);

    // 2. Set the decryption key (for CTR mode, you use the encryption key schedule)
    // Meshtastic uses AES-256, so the key is 256 bits (32 bytes).
    int ret = mbedtls_aes_setkey_enc(&aes_ctx, key, 256);
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_aes_setkey_enc failed with error: -0x%04x", -ret);
        mbedtls_aes_free(&aes_ctx);
        return false;
    }

    // 3. Prepare the nonce, stream block, and offset for AES-CTR
    uint8_t nonce[16];
    uint8_t stream_block[16];
    size_t nc_off = 0;  // Offset in the current stream block, always start at 0

    generate_meshtastic_nonce(packet_id, from_node, nonce);

    // 4. Perform the decryption
    ret = mbedtls_aes_crypt_ctr(&aes_ctx, len, &nc_off, nonce, stream_block, encrypted_in, decrypted_out);
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_aes_crypt_ctr failed with error: -0x%04x", -ret);
        mbedtls_aes_free(&aes_ctx);
        return false;
    }

    // 5. Clean up the context
    mbedtls_aes_free(&aes_ctx);

    return true;
}