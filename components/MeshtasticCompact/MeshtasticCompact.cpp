
#include "MeshtasticCompact.hpp"
#include "esp_log.h"
#include "meshtastic/mesh.pb.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "unishox2.h"
#include "meshtastic/remote_hardware.pb.h"
#include "meshtastic/telemetry.pb.h"
#include "esp_mac.h"
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
    uint64_t mac;
    esp_efuse_mac_get_default((uint8_t*)&mac);  // Use the last 4 bytes of the MAC address as the node ID
    my_id = (uint32_t)(mac & 0xFFFFFFFF);
    RadioListen();  // Start listening for packets
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
                mshcomp->ProcessPacket(rxData, rxLen, mshcomp);
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

void MeshtasticCompact::intOnMessage(MC_Header header, MC_TextMessage message) {
    // we won't cache, it is the upper layer's thing.
    if (onMessage) {
        onMessage(header, message);
    };
}

void MeshtasticCompact::intOnPositionMessage(MC_Header header, MC_Position position) {
    // should save this to contact's data //todo
    if (onPositionMessage) {
        onPositionMessage(header, position);
    };
}

int16_t MeshtasticCompact::ProcessPacket(uint8_t* data, int len, MeshtasticCompact* mshcomp) {
    if (len > 0) {
        // https://meshtastic.org/docs/overview/mesh-algo/#layer-1-unreliable-zero-hop-messaging
        if (len < 0x10) {
            ESP_LOGE(TAG, "Received packet too short: %d bytes", len);
            return false;
        }
        MC_Header header;
        header.rssi = mshcomp->rssi = mshcomp->radio.getRSSI();
        header.snr = mshcomp->snr = mshcomp->radio.getSNR();

        memcpy(&header.dstnode, &data[0], sizeof(uint32_t));
        memcpy(&header.srcnode, &data[4], sizeof(uint32_t));
        memcpy(&header.packet_id, &data[8], sizeof(uint32_t));

        uint8_t packet_flags = data[12];
        header.chan_hash = data[13];
        // uint8_t packet_next_hop = data[14];
        // uint8_t packet_relay_node = data[15];

        // extract flags  https://github.com/meshtastic/firmware/blob/e505ec847e20167ceca273fe22872720a5df7439/src/mesh/RadioLibInterface.cpp#L453
        header.hop_limit = packet_flags & PACKET_FLAGS_HOP_LIMIT_MASK;
        header.want_ack = !!(packet_flags & PACKET_FLAGS_WANT_ACK_MASK);
        header.via_mqtt = !!(packet_flags & PACKET_FLAGS_VIA_MQTT_MASK);
        header.hop_start = (packet_flags & PACKET_FLAGS_HOP_START_MASK) >> PACKET_FLAGS_HOP_START_SHIFT;

        meshtastic_Data decodedtmp;
        int16_t ret = try_decode_root_packet(&data[16], len - 16, &meshtastic_Data_msg, &decodedtmp, sizeof(decodedtmp), header.packet_id, header.srcnode);
        if (ret >= 0) {
            ESP_LOGI(TAG, "Decoded Meshtastic Data:");
            ESP_LOGI(TAG, "PortNum: %d", decodedtmp.portnum);
            // ESP_LOGI(TAG, "Payload: %s", decodedtmp.payload.bytes);
            ESP_LOGI(TAG, "Want Response: %d", decodedtmp.want_response);
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
                intOnMessage(header, {std::string(reinterpret_cast<const char*>(decodedtmp.payload.bytes), decodedtmp.payload.size), (uint8_t)ret, MC_MESSAGE_TYPE_TEXT});
            } else if (decodedtmp.portnum == 2) {
                ESP_LOGI(TAG, "Received a remote hardware packet");
                // payload: protobuf HardwareMessage - NOT INTERESTED IN YET
                /*meshtastic_HardwareMessage hardware_msg = {};
                if (pb_decode_from_bytes(decodedtmp.payload.bytes, decodedtmp.payload.size, &meshtastic_HardwareMessage_msg, &hardware_msg)) {
                    ESP_LOGI(TAG, "Hardware Message Type: %d", hardware_msg.type);
                    ESP_LOGI(TAG, "GPIO Mask: 0x%016llX", hardware_msg.gpio_mask);
                    ESP_LOGI(TAG, "GPIO Value: 0x%016llX", hardware_msg.gpio_value);
                } else {
                    ESP_LOGE(TAG, "Failed to decode HardwareMessage");
                }*/
            } else if (decodedtmp.portnum == 3) {
                ESP_LOGI(TAG, "Received a position packet");
                // payload: protobuf Position
                meshtastic_Position position_msg = {};  // todo add callback
                if (pb_decode_from_bytes(decodedtmp.payload.bytes, decodedtmp.payload.size, &meshtastic_Position_msg, &position_msg)) {
                    intOnPositionMessage(header, {.latitude_i = position_msg.latitude_i,
                                                  .longitude_i = position_msg.longitude_i,
                                                  .altitude = position_msg.altitude,
                                                  .ground_speed = position_msg.ground_speed,
                                                  .sats_in_view = position_msg.sats_in_view,
                                                  .location_source = (uint8_t)position_msg.location_source});
                } else {
                    ESP_LOGE(TAG, "Failed to decode Position");
                }
            } else if (decodedtmp.portnum == 4) {
                ESP_LOGI(TAG, "Received a node info packet");
                // payload: protobuf User
                meshtastic_User user_msg = {};  // todo store, and callback
                if (pb_decode_from_bytes(decodedtmp.payload.bytes, decodedtmp.payload.size, &meshtastic_User_msg, &user_msg)) {
                    ESP_LOGI(TAG, "User ID: %s", user_msg.id);
                    ESP_LOGI(TAG, "Shortname: %s", user_msg.short_name);
                    ESP_LOGI(TAG, "Longname: %s", user_msg.long_name);
                    ESP_LOGI(TAG, "HW Model: %d", user_msg.hw_model);
                    // ESP_LOGI(TAG, "HW Model: %s", user_msg.macaddr);
                    // ESP_LOGI(TAG, "HW Model: %s", user_msg.public_key);
                    ESP_LOGI(TAG, "Role: %d", user_msg.role);
                } else {
                    ESP_LOGE(TAG, "Failed to decode User");
                }
            } else if (decodedtmp.portnum == 5) {
                ESP_LOGI(TAG, "Received a routing packet");
                // payload: protobuf Routing
                meshtastic_Routing routing_msg = {};  // todo process it. this is just a debug. or simply drop it.
                if (pb_decode_from_bytes(decodedtmp.payload.bytes, decodedtmp.payload.size, &meshtastic_Routing_msg, &routing_msg)) {
                    ESP_LOGI(TAG, "Routing reply count: %d", routing_msg.route_reply.route_count);
                } else {
                    ESP_LOGE(TAG, "Failed to decode Routing");
                }
            } else if (decodedtmp.portnum == 6) {
                ESP_LOGI(TAG, "Received an admin packet");
                // payload: protobuf AdminMessage
                // drop it, not interested in admin messages
            } else if (decodedtmp.portnum == 7) {
                ESP_LOGI(TAG, "Received a compressed text message packet");
                // payload: utf8 text with Unishox2 Compression
                char uncompressed_data[256] = {0};
                size_t uncompressed_size = unishox2_decompress((const char*)&decodedtmp.payload.bytes, decodedtmp.payload.size, uncompressed_data, sizeof(uncompressed_data), USX_PSET_DFLT);
                intOnMessage(header, {std::string(reinterpret_cast<const char*>(uncompressed_data), uncompressed_size), (uint8_t)ret, MC_MESSAGE_TYPE_TEXT});
            } else if (decodedtmp.portnum == 8) {
                ESP_LOGI(TAG, "Received a waypoint packet");
                // payload: protobuf Waypoint
                meshtastic_Waypoint waypoint_msg = {};  // todo store and callback
                if (pb_decode_from_bytes(decodedtmp.payload.bytes, decodedtmp.payload.size, &meshtastic_Waypoint_msg, &waypoint_msg)) {
                    ESP_LOGI(TAG, "Waypoint ID: %lu", waypoint_msg.id);
                    ESP_LOGI(TAG, "Name: %s", waypoint_msg.name);
                    ESP_LOGI(TAG, "Description: %s", waypoint_msg.description);
                    ESP_LOGI(TAG, "Latitude: %ld", waypoint_msg.latitude_i);
                    ESP_LOGI(TAG, "Longitude: %ld", waypoint_msg.longitude_i);
                    ESP_LOGI(TAG, "Altitude: %lu", waypoint_msg.icon);
                    ESP_LOGI(TAG, "Description: %lu", waypoint_msg.expire);
                } else {
                    ESP_LOGE(TAG, "Failed to decode Waypoint");
                }
            } else if (decodedtmp.portnum == 10) {
                ESP_LOGI(TAG, "Received a detection sensor packet");
                // payload: utf8 text
                intOnMessage(header, {std::string(reinterpret_cast<const char*>(decodedtmp.payload.bytes), decodedtmp.payload.size), (uint8_t)ret, MC_MESSAGE_TYPE_DETECTOR_SENSOR});
            } else if (decodedtmp.portnum == 11) {
                ESP_LOGI(TAG, "Received an alert packet");
                // payload: utf8 text
                intOnMessage(header, {std::string(reinterpret_cast<const char*>(decodedtmp.payload.bytes), decodedtmp.payload.size), (uint8_t)ret, MC_MESSAGE_TYPE_ALERT});
            } else if (decodedtmp.portnum == 12) {
                ESP_LOGI(TAG, "Received a key verification packet");
                // payload: protobuf KeyVerification
                meshtastic_KeyVerification key_verification_msg = {};  // todo drop?
                if (pb_decode_from_bytes(decodedtmp.payload.bytes, decodedtmp.payload.size, &meshtastic_KeyVerification_msg, &key_verification_msg)) {
                    ;
                } else {
                    ESP_LOGE(TAG, "Failed to decode KeyVerification");
                }
            } else if (decodedtmp.portnum == 32) {
                ESP_LOGI(TAG, "Received a reply packet");
                // payload: ASCII Plaintext //TODO determine the in/out part and send reply if needed
                intOnMessage(header, {std::string(reinterpret_cast<const char*>(decodedtmp.payload.bytes), decodedtmp.payload.size), (uint8_t)ret, MC_MESSAGE_TYPE_PING});
            } else if (decodedtmp.portnum == 34) {
                ESP_LOGI(TAG, "Received a paxcounter packet");
                // payload: protobuf DROP
            } else if (decodedtmp.portnum == 64) {
                ESP_LOGI(TAG, "Received a serial packet");
                // payload: uart rx/tx data
                intOnMessage(header, {std::string(reinterpret_cast<const char*>(decodedtmp.payload.bytes), decodedtmp.payload.size), (uint8_t)ret, MC_MESSAGE_TYPE_UART});
            } else if (decodedtmp.portnum == 65) {
                ESP_LOGI(TAG, "Received a STORE_FORWARD_APP  packet");
                // payload: ?
            } else if (decodedtmp.portnum == 66) {
                ESP_LOGI(TAG, "Received a RANGE_TEST_APP  packet");
                // payload: ascii text
                intOnMessage(header, {std::string(reinterpret_cast<const char*>(decodedtmp.payload.bytes), decodedtmp.payload.size), (uint8_t)ret, MC_MESSAGE_TYPE_RANGE_TEST});
            } else if (decodedtmp.portnum == 67) {
                ESP_LOGI(TAG, "Received a TELEMETRY_APP   packet");
                // payload: Protobuf
                meshtastic_Telemetry telemetry_msg = {};  // todo store and callback
                if (pb_decode_from_bytes(decodedtmp.payload.bytes, decodedtmp.payload.size, &meshtastic_Telemetry_msg, &telemetry_msg)) {
                    ESP_LOGI(TAG, "Telemetry Time: %lu", telemetry_msg.time);
                    switch (telemetry_msg.which_variant) {
                        case meshtastic_Telemetry_device_metrics_tag:
                            ESP_LOGI(TAG, "Device Metrics: Battery Level: %lu", telemetry_msg.variant.device_metrics.battery_level);
                            break;
                        case meshtastic_Telemetry_environment_metrics_tag:
                            ESP_LOGI(TAG, "Environment Metrics: Temperature: %f", telemetry_msg.variant.environment_metrics.temperature);
                            break;
                        case meshtastic_Telemetry_air_quality_metrics_tag:
                            ESP_LOGI(TAG, "Air Quality Metrics: PM2.5: %lu ", telemetry_msg.variant.air_quality_metrics.pm25_standard);
                            break;
                        case meshtastic_Telemetry_power_metrics_tag:
                            break;
                        case meshtastic_Telemetry_local_stats_tag:
                            ESP_LOGI(TAG, "Local Stats: Packets Sent: %lu, Packets Received: %lu", telemetry_msg.variant.local_stats.num_packets_tx, telemetry_msg.variant.local_stats.num_packets_rx);
                            break;
                        case meshtastic_Telemetry_health_metrics_tag:
                            ESP_LOGI(TAG, "Health Metrics: Hearth BPM: %u, Temp: %f  So2: %u", telemetry_msg.variant.health_metrics.heart_bpm, telemetry_msg.variant.health_metrics.temperature, telemetry_msg.variant.health_metrics.spO2);
                            break;
                        case meshtastic_Telemetry_host_metrics_tag:
                            break;
                    };
                } else {
                    ESP_LOGE(TAG, "Failed to decode Telemetry");
                }
            } else if (decodedtmp.portnum == 70) {
                ESP_LOGI(TAG, "Received a TRACEROUTE_APP    packet");
                // payload: Protobuf RouteDiscovery
                meshtastic_RouteDiscovery route_discovery_msg = {};  // drop
                if (pb_decode_from_bytes(decodedtmp.payload.bytes, decodedtmp.payload.size, &meshtastic_RouteDiscovery_msg, &route_discovery_msg)) {
                    ESP_LOGI(TAG, "Route Discovery: Hop Count: %d", route_discovery_msg.route_count);
                } else {
                    ESP_LOGE(TAG, "Failed to decode RouteDiscovery");
                }
            } else if (decodedtmp.portnum == 71) {
                ESP_LOGI(TAG, "Received a NEIGHBORINFO_APP   packet");
                // payload: Protobuf ?
            } else {
                ESP_LOGI(TAG, "Received an unhandled portnum: %d", decodedtmp.portnum);
            }
            if (header.want_ack && is_send_enabled) {
                // todo send ack
            }
        }
        return ret;
    }
    return false;
}

/*

DECODER HELPERS

*/

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

int16_t MeshtasticCompact::try_decode_root_packet(const uint8_t* srcbuf, size_t srcbufsize, const pb_msgdesc_t* fields, void* dest_struct, size_t dest_struct_size, uint32_t packet_id, uint32_t packet_src) {
    uint8_t decrypted_data[srcbufsize] = {0};
    memset(dest_struct, 0, dest_struct_size);
    // 1st.
    if (aes_decrypt_meshtastic_payload(default_l1_key, sizeof(default_l1_key) * 8, packet_id, packet_src, srcbuf, decrypted_data, srcbufsize)) {
        if (pb_decode_from_bytes(decrypted_data, srcbufsize, fields, dest_struct)) return 254;
    }
    memset(dest_struct, 0, dest_struct_size);
    if (aes_decrypt_meshtastic_payload(default_chan_key, sizeof(default_chan_key) * 8, packet_id, packet_src, srcbuf, decrypted_data, srcbufsize)) {
        if (pb_decode_from_bytes(decrypted_data, srcbufsize, fields, dest_struct)) return 0;
    }
    // todo iterate chan keys

    ESP_LOGI(TAG, "can't decode packet");
    return -1;
}