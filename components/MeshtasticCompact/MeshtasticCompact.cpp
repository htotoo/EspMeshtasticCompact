
#include "MeshtasticCompact.hpp"
#include "esp_log.h"
#include "meshtastic/mesh.pb.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

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
MeshtasticCompact::MeshtasticCompact() {
    mbedtls_aes_init(&aes_ctx);
}

MeshtasticCompact::~MeshtasticCompact() {
    mbedtls_aes_free(&aes_ctx);
}

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
            mshcomp->radio.startReceive();
            if (err >= 0) {
                float rssi, snr;
                rssi = mshcomp->radio.getRSSI();
                snr = mshcomp->radio.getSNR();
                printf("rssi=%f[dBm] snr=%f[dB]\n", rssi, snr);
                mshcomp->ProcessPacket(rxData, rxLen);
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
        vTaskDelay(1);
    }  // end while
    // never reach here
    vTaskDelete(NULL);
}

bool MeshtasticCompact::RadioListen() {
    xTaskCreate(&task_listen, "RadioListen", 1024 * 4, this, 5, NULL);
    return true;
}

bool MeshtasticCompact::ProcessPacket(uint8_t* data, int len) {
    if (len > 0) {
        // https://meshtastic.org/docs/overview/mesh-algo/#layer-1-unreliable-zero-hop-messaging
        if (len < 0x10) {
            ESP_LOGE(TAG, "Received packet too short: %d bytes", len);
            return false;
        }
        uint32_t packet_dest, packet_src, packet_id;
        memcpy(&packet_dest, &data[0], sizeof(uint32_t));
        memcpy(&packet_src, &data[4], sizeof(uint32_t));
        memcpy(&packet_id, &data[8], sizeof(uint32_t));

        uint8_t packet_flags = data[12];
        uint8_t packet_chan_hash = data[13];
        uint8_t packet_next_hop = data[14];
        uint8_t packet_relay_node = data[15];

        // ESP_LOGI(TAG, "Received packet: dest=0x%08lX, src=0x%08lX, id=%" PRIu32 ", flags=0x%02X, chan_hash=0x%02X, next_hop=%d, relay_node=%d", packet_dest, packet_src, packet_id, packet_flags, packet_chan_hash, packet_next_hop, packet_relay_node);
        //  extract flags  https://github.com/meshtastic/firmware/blob/e505ec847e20167ceca273fe22872720a5df7439/src/mesh/RadioLibInterface.cpp#L453
        uint8_t packet_hop_limit = packet_flags & PACKET_FLAGS_HOP_LIMIT_MASK;
        bool packet_want_Ack = !!(packet_flags & PACKET_FLAGS_WANT_ACK_MASK);
        bool packet_mqtt = !!(packet_flags & PACKET_FLAGS_VIA_MQTT_MASK);
        uint8_t packet_hop_start = (packet_flags & PACKET_FLAGS_HOP_START_MASK) >> PACKET_FLAGS_HOP_START_SHIFT;
        // ESP_LOGI(TAG, "Header flags: hop_limit=%u, want_ack=%d, mqtt=%d, hopstart=%u", packet_hop_limit, packet_want_Ack, packet_mqtt, packet_hop_start);

        meshtastic_Data decodedtmp;
        bool ret = try_decode_root_packet(&data[16], len - 16, &meshtastic_Data_msg, &decodedtmp, sizeof(decodedtmp), packet_id, packet_src);
        if (ret) {
            ESP_LOGI(TAG, "Decoded Meshtastic Data:");
            ESP_LOGI(TAG, "PortNum: %d", decodedtmp.portnum);
            // ESP_LOGI(TAG, "Payload: %s", decodedtmp.payload.bytes);
            ESP_LOGI(TAG, "Want Response: %d", decodedtmp.want_response);
            ESP_LOGI(TAG, "Dest: 0x%08lX", decodedtmp.dest);
            ESP_LOGI(TAG, "Source: 0x%08lX", decodedtmp.source);
            ESP_LOGI(TAG, "Request ID: %" PRIu32, decodedtmp.request_id);
            ESP_LOGI(TAG, "Reply ID: %" PRIu32, decodedtmp.reply_id);
            ESP_LOGI(TAG, "Emoji: %" PRIu32, decodedtmp.emoji);
            ESP_LOGI(TAG, "Bitfield: 0x%02X", decodedtmp.bitfield);
            // Process the decoded data as needed https://github.com/meshtastic/protobufs/blob/master/meshtastic/portnums.proto
            if (decodedtmp.portnum == 0) {
                ESP_LOGI(TAG, "Received an unknown packet");
            } else if (decodedtmp.portnum == 1) {
                ESP_LOGI(TAG, "Received a message packet");
                // payload: utf8 text
            } else if (decodedtmp.portnum == 2) {
                ESP_LOGI(TAG, "Received a remote hardware packet");
                // payload: protobuf HardwareMessage
            } else if (decodedtmp.portnum == 3) {
                ESP_LOGI(TAG, "Received a position packet");
                // payload: protobuf Position
            } else if (decodedtmp.portnum == 4) {
                ESP_LOGI(TAG, "Received a node info packet");
                // payload: protobuf User
            } else if (decodedtmp.portnum == 5) {
                ESP_LOGI(TAG, "Received a routing packet");
                // payload: protobuf Routing
            } else if (decodedtmp.portnum == 6) {
                ESP_LOGI(TAG, "Received an admin packet");
                // payload: protobuf AdminMessage
            } else if (decodedtmp.portnum == 7) {
                ESP_LOGI(TAG, "Received a compressed text message packet");
                // payload: utf8 text with Unishox2 Compression
            } else if (decodedtmp.portnum == 8) {
                ESP_LOGI(TAG, "Received a waypoint packet");
                // payload: protobuf Waypoint
            } else if (decodedtmp.portnum == 9) {
                ESP_LOGI(TAG, "Received an audio packet");
                // payload: codec2 audio frames
            } else if (decodedtmp.portnum == 10) {
                ESP_LOGI(TAG, "Received a detection sensor packet");
                // payload: utf8 text
            } else if (decodedtmp.portnum == 11) {
                ESP_LOGI(TAG, "Received an alert packet");
                // payload: utf8 text
            } else if (decodedtmp.portnum == 12) {
                ESP_LOGI(TAG, "Received a key verification packet");
                // payload: protobuf KeyVerification
            } else if (decodedtmp.portnum == 32) {
                ESP_LOGI(TAG, "Received a reply packet");
                // payload: ASCII Plaintext
            } else if (decodedtmp.portnum == 34) {
                ESP_LOGI(TAG, "Received a paxcounter packet");
                // payload: protobuf
            } else if (decodedtmp.portnum == 64) {
                ESP_LOGI(TAG, "Received a serial packet");
                // payload: uart rx/tx data
            } else if (decodedtmp.portnum == 65) {
                ESP_LOGI(TAG, "Received a STORE_FORWARD_APP  packet");
                // payload: ?
            } else if (decodedtmp.portnum == 66) {
                ESP_LOGI(TAG, "Received a RANGE_TEST_APP  packet");
                // payload: ascii text
            } else if (decodedtmp.portnum == 67) {
                ESP_LOGI(TAG, "Received a TELEMETRY_APP   packet");
                // payload: Protobuf tocheck
            } else if (decodedtmp.portnum == 70) {
                ESP_LOGI(TAG, "Received a TRACEROUTE_APP    packet");
                // payload: Protobuf RouteDiscovery
            } else if (decodedtmp.portnum == 71) {
                ESP_LOGI(TAG, "Received a NEIGHBORINFO_APP   packet");
                // payload: Protobuf ?
            } else {
                ESP_LOGI(TAG, "Received an unhandled portnum: %d", decodedtmp.portnum);
            }
            return ret;
        }
    }
    return false;
}

bool MeshtasticCompact::aes_decrypt_meshtastic_payload(const uint8_t* key, uint16_t keySize, uint32_t packet_id, uint32_t from_node, const uint8_t* encrypted_in, uint8_t* decrypted_out, size_t len) {
    int ret = mbedtls_aes_setkey_enc(&aes_ctx, key, keySize);
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_aes_setkey_enc failed with error: -0x%04x", -ret);
        mbedtls_aes_free(&aes_ctx);
        return false;
    }
    uint8_t nonce[16];
    uint8_t stream_block[16];
    size_t nc_off = 0;
    memset(nonce, 0, 16);
    memcpy(nonce, &packet_id, sizeof(uint32_t));
    memcpy(nonce + 8, &from_node, sizeof(uint32_t));
    ret = mbedtls_aes_crypt_ctr(&aes_ctx, len, &nc_off, nonce, stream_block, encrypted_in, decrypted_out);
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_aes_crypt_ctr failed with error: -0x%04x", -ret);
        mbedtls_aes_free(&aes_ctx);
        return false;
    }
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

size_t MeshtasticCompact::pb_encode_to_bytes(uint8_t* destbuf, size_t destbufsize, const pb_msgdesc_t* fields, const void* src_struct) {
    pb_ostream_t stream = pb_ostream_from_buffer(destbuf, destbufsize);
    if (!pb_encode(&stream, fields, src_struct)) {
        printf("Panic: can't encode protobuf reason='%s'", PB_GET_ERROR(&stream));
        return 0;
    } else {
        return stream.bytes_written;
    }
}

bool MeshtasticCompact::try_decode_root_packet(const uint8_t* srcbuf, size_t srcbufsize, const pb_msgdesc_t* fields, void* dest_struct, size_t dest_struct_size, uint32_t packet_id, uint32_t packet_src) {
    uint8_t decrypted_data[srcbufsize] = {0};
    memset(dest_struct, 0, dest_struct_size);
    // 1st.
    if (aes_decrypt_meshtastic_payload(default_l1_key, sizeof(default_l1_key) * 8, packet_id, packet_src, srcbuf, decrypted_data, srcbufsize)) {
        if (pb_decode_from_bytes(decrypted_data, srcbufsize, fields, dest_struct)) return true;
    }
    memset(dest_struct, 0, dest_struct_size);
    if (aes_decrypt_meshtastic_payload(default_chan_key, sizeof(default_chan_key) * 8, packet_id, packet_src, srcbuf, decrypted_data, srcbufsize)) {
        if (pb_decode_from_bytes(decrypted_data, srcbufsize, fields, dest_struct)) return true;
    }
    // todo iterate chan keys

    ESP_LOGI(TAG, "can't decode packet");
    return false;
}