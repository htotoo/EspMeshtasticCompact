
#include "MeshtasticCompact.hpp"
#include "esp_log.h"
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

#define BITFIELD_WANT_RESPONSE_SHIFT 1
#define BITFIELD_OK_TO_MQTT_SHIFT 0
#define BITFIELD_WANT_RESPONSE_MASK (1 << BITFIELD_WANT_RESPONSE_SHIFT)
#define BITFIELD_OK_TO_MQTT_MASK (1 << BITFIELD_OK_TO_MQTT_SHIFT)
#define NODEINFO_BITFIELD_IS_KEY_MANUALLY_VERIFIED_SHIFT 0
#define NODEINFO_BITFIELD_IS_KEY_MANUALLY_VERIFIED_MASK (1 << NODEINFO_BITFIELD_IS_KEY_MANUALLY_VERIFIED_SHIFT)

volatile bool packetFlag = false;

static_assert(CONFIG_ESP_MAIN_TASK_STACK_SIZE >= 8000, "Main task stack size must be at least 8000 bytes!");

void IRAM_ATTR onPacketReceived() {
    packetFlag = true;
}
MeshtasticCompact::MeshtasticCompact() {
    mbedtls_aes_init(&aes_ctx);
    uint64_t mac;
    esp_efuse_mac_get_default((uint8_t*)&mac);  // Use the last 4 bytes of the MAC address as the node ID
    my_nodeinfo.node_id = (uint32_t)(mac & 0xFFFFFFFF);
    sprintf(my_nodeinfo.id, "!0x%08" PRIx32, my_nodeinfo.node_id);
    my_nodeinfo.hw_model = (uint8_t)meshtastic_HardwareModel_DIY_V1;
    my_nodeinfo.role = (uint8_t)meshtastic_Config_DeviceConfig_Role_CLIENT;
    my_nodeinfo.macaddr[0] = (uint8_t)(mac >> 40);
    my_nodeinfo.macaddr[1] = (uint8_t)(mac >> 32);
    my_nodeinfo.macaddr[2] = (uint8_t)(mac >> 24);
    my_nodeinfo.macaddr[3] = (uint8_t)(mac >> 16);
    my_nodeinfo.macaddr[4] = (uint8_t)(mac >> 8);
    my_nodeinfo.macaddr[5] = (uint8_t)(mac & 0xFF);
    my_nodeinfo.public_key_size = 0;                                    // Set to 0 if no public key is available
    memset(my_nodeinfo.public_key, 0, sizeof(my_nodeinfo.public_key));  // Initialize public key to zero
    sprintf(my_nodeinfo.short_name, "MCP");
    snprintf(my_nodeinfo.long_name, sizeof(my_nodeinfo.long_name) - 1, "MeshtasticCompact-%02" PRIx32, my_nodeinfo.node_id);
    router.setMyId(my_nodeinfo.node_id);
}

MeshtasticCompact::~MeshtasticCompact() {
    mbedtls_aes_free(&aes_ctx);
    need_run = false;  // Stop the tasks
    packetFlag = true;
    out_queue.stop_wait();                 // Notify any waiting pop() to unblock immediately
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Give some time for tasks to finish
}

bool MeshtasticCompact::RadioInit(Radio_PINS& radio_pins, LoraConfig& lora_config) {
    ESP_LOGI(TAG, "RadioInit");
    hal = new EspHal(radio_pins.sck, radio_pins.miso, radio_pins.mosi);
    radio = new SX1262(new Module(hal, radio_pins.cs, radio_pins.irq, radio_pins.rst, radio_pins.gpio));

    ESP_LOGI(TAG, "Init");
    int state = ((SX1262*)radio)->begin(lora_config.frequency, lora_config.bandwidth, lora_config.spreading_factor, lora_config.coding_rate, lora_config.sync_word, lora_config.output_power, lora_config.preamble_length, lora_config.tcxo_voltage, lora_config.use_regulator_ldo);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "failed, code %d\n", state);
        while (true) {
            hal->delay(1000);
        }
    }
    ESP_LOGI(TAG, "success!\n");
    state |= ((SX1262*)radio)->setCurrentLimit(130.0);
    state |= ((SX1262*)radio)->explicitHeader();
    state |= ((SX1262*)radio)->setCRC(RADIOLIB_SX126X_LORA_CRC_ON);
    state |= ((SX1262*)radio)->setDio2AsRfSwitch(false);
    ((SX1262*)radio)->setDio1Action(onPacketReceived);
    state |= ((SX1262*)radio)->setRxBoostedGainMode(true);
    if (state != 0) {
        ESP_LOGE(TAG, "Radio init failed, code %d\n", state);
        return false;
    }
    RadioListen();    // Start listening for packets
    RadioSendInit();  // Start the send task
    return true;
}

void MeshtasticCompact::task_send(void* pvParameters) {
    MeshtasticCompact* mshcomp = static_cast<MeshtasticCompact*>(pvParameters);
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    while (mshcomp->need_run) {
        MC_OutQueueEntry entry = mshcomp->out_queue.pop();

        if (entry.header.srcnode == 0) {
            // Stop flag was set, exit the task
            ESP_LOGI(TAG, "Send task stopped");
            continue;
        }
        if (mshcomp->is_send_enabled) {
            // packet id fix:
            if (entry.header.packet_id == 0) {
                entry.header.packet_id = mshcomp->hal->millis() + 100000;
            }
            // Prepare the payload
            uint8_t payload[256];
            uint8_t relay_node = 0;
            // set some fields when it is from me
            if (entry.header.srcnode == mshcomp->my_nodeinfo.node_id) {
                entry.data.bitfield |= 1 << BITFIELD_OK_TO_MQTT_SHIFT;  // Set the MQTT upload bit
                entry.data.bitfield |= 1 << (entry.data.want_response << BITFIELD_WANT_RESPONSE_SHIFT);
                entry.data.has_bitfield = true;
            } else {
                relay_node = mshcomp->getLastByteOfNodeNum(mshcomp->my_nodeinfo.node_id);
            }

            size_t payload_len = mshcomp->pb_encode_to_bytes(payload, sizeof(payload), meshtastic_Data_fields, &entry.data);
            // Encrypt the payload if needed
            uint8_t encrypted_payload[256];
            bool aesenc = true;
            if (entry.encType == 0) {  // the auto enc method
                MC_NodeInfo* dstnode = mshcomp->nodeinfo_db.get(entry.header.dstnode);
                if (entry.header.chan_hash == 0 && entry.header.dstnode != 0xffffffff && dstnode && (dstnode->public_key_size == 16 || dstnode->public_key_size == 32)) {
                    bool all_zero = true;
                    for (size_t i = 0; i < dstnode->public_key_size; i++) {
                        if (dstnode->public_key[i] != 0) {
                            all_zero = false;
                            break;
                        }
                    }
                    if (!all_zero) {
                        aesenc = false;
                    }
                }
            }

            if (entry.encType == 1)
                aesenc = true;  // AES encryption
            else if (entry.encType == 2)
                aesenc = false;  // key

            if (!aesenc) {
                // private message, encrypt with that method if pubkey is availeable //todo
                continue;
            } else {
                if (mshcomp->aes_decrypt_meshtastic_payload(entry.key, entry.key_len * 8, entry.header.packet_id, entry.header.srcnode, payload, encrypted_payload, payload_len)) {
                } else {
                    ESP_LOGE(TAG, "Failed to encrypt payload");
                    continue;
                }
            }
            // here we got the encrypted payload
            // create header. ugly but we'll reuse the payload buffer
            memcpy(&payload[0], &entry.header.dstnode, sizeof(uint32_t));
            memcpy(&payload[4], &entry.header.srcnode, sizeof(uint32_t));
            memcpy(&payload[8], &entry.header.packet_id, sizeof(uint32_t));
            payload[12] = (entry.header.hop_limit & PACKET_FLAGS_HOP_LIMIT_MASK) | (entry.header.want_ack ? PACKET_FLAGS_WANT_ACK_MASK : 0) | (entry.header.via_mqtt ? PACKET_FLAGS_VIA_MQTT_MASK : 0) | ((entry.header.hop_start & 0x07) << PACKET_FLAGS_HOP_START_SHIFT);
            payload[13] = entry.header.chan_hash;
            payload[14] = 0;           // entry.header.packet_next_hop; -- no preference
            payload[15] = relay_node;  // entry.header.packet_relay_node;
            // copy the encrypted payload to the end of the header
            size_t total_len = 16 + payload_len;  // 16 bytes for header + payload length
            if (total_len > sizeof(payload)) {
                ESP_LOGE(TAG, "Payload too large: %zu bytes", total_len);
                continue;
            }
            memcpy(&payload[16], encrypted_payload, payload_len);
            // Send the packet
            {
                std::unique_lock<std::mutex> lock(mshcomp->mtx_radio);
                ESP_LOGE(TAG, "Try send packet");
                int err = mshcomp->radio->transmit(payload, total_len);
                if (err == RADIOLIB_ERR_NONE) {
                    ESP_LOGI(TAG, "Packet sent successfully to node 0x%08" PRIx32 ", ID: 0x%08" PRIx32, entry.header.dstnode, entry.header.packet_id);
                } else {
                    ESP_LOGE(TAG, "Failed to send packet, code %d", err);
                    vTaskDelay(30 / portTICK_PERIOD_MS);
                    err = mshcomp->radio->transmit(payload, total_len);
                    if (err == RADIOLIB_ERR_NONE) {
                        ESP_LOGI(TAG, "Packet sent successfully in 2nd try to node 0x%08" PRIx32 ", ID: 0x%08" PRIx32, entry.header.dstnode, entry.header.packet_id);
                    } else {
                        ESP_LOGE(TAG, "Failed to send packet 2 times in a row, code %d", err);
                    }
                }
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
        mshcomp->radio->startReceive();
        // Restart receiving after sending
        vTaskDelay(350 / portTICK_PERIOD_MS);  // Wait before next send attempt
    }  // end while
    // never reach here
    vTaskDelete(NULL);
}

void MeshtasticCompact::task_listen(void* pvParameters) {
    MeshtasticCompact* mshcomp = static_cast<MeshtasticCompact*>(pvParameters);
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    uint8_t rxData[256];  // Maximum Payload size of SX1261/62/68 is 255
    mshcomp->radio->startReceive();
    while (mshcomp->need_run) {
        if (packetFlag) {
            if (!mshcomp->need_run) break;
            packetFlag = false;
            int err = 0;
            int rxLen = 0;
            {
                std::unique_lock<std::mutex> lock(mshcomp->mtx_radio);
                // ESP_LOGW(TAG, "Packet received, trying to read data");
                rxLen = mshcomp->radio->getPacketLength();
                if (rxLen > 255) rxLen = 255;  // Ensure we do not overflow the buffer
                err = mshcomp->radio->readData(rxData, rxLen);
                mshcomp->rssi = mshcomp->radio->getRSSI();
                mshcomp->snr = mshcomp->radio->getSNR();
                vTaskDelay(1 / portTICK_PERIOD_MS);
                mshcomp->radio->startReceive();
            }
            if (err >= 0) {
                if (mshcomp->onRaw) {
                    mshcomp->onRaw(rxData, rxLen);
                }
                mshcomp->ProcessPacket(rxData, rxLen, mshcomp);
            }

            if (err < 0) {
                if (err == RADIOLIB_ERR_RX_TIMEOUT) {
                    // timeout occurred while waiting for a packet
                    // printf("timeout!\n");
                } else if (err == RADIOLIB_ERR_CRC_MISMATCH) {
                    // packet was received, but is malformed
                    // printf("CRC error!\n");
                } else {
                    // some other error occurred
                    // printf("failed, code %d", err);
                }
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);  // Wait before next receive attempt
    }  // end while
    // never reach here
    vTaskDelete(NULL);
}

bool MeshtasticCompact::RadioListen() {
    xTaskCreate(&task_listen, "RadioListen", 1024 * 4, this, 5, NULL);
    return true;
}

bool MeshtasticCompact::RadioSendInit() {
    // Start the send task
    xTaskCreate(&task_send, "RadioSend", 1024 * 4, this, 5, NULL);
    return true;
}
void MeshtasticCompact::intOnMessage(MC_Header& header, MC_TextMessage& message) {
    // we won't cache, it is the upper layer's thing.
    if (onMessage) {
        onMessage(header, message);
    };
}

void MeshtasticCompact::intOnPositionMessage(MC_Header& header, MC_Position& position, bool want_reply) {
    if (position.has_latitude_i && position.has_longitude_i) nodeinfo_db.setPosition(header.srcnode, position);  // not saved the request, since that is mostly empty
    bool needReply = (want_reply == true && !is_auto_full_node && is_send_enabled);
    if (onPositionMessage) {
        onPositionMessage(header, position, needReply);
    };
    if (want_reply && is_auto_full_node && is_send_enabled) {
        ESP_LOGI(TAG, "AUTO Sending my pos info to node 0x%08" PRIx32, header.srcnode);
        SendMyPosition(header.srcnode);
    }
}

void MeshtasticCompact::intOnNodeInfo(MC_Header& header, MC_NodeInfo& nodeinfo, bool want_reply) {
    nodeinfo_db.addOrUpdate(header.srcnode, nodeinfo);  // if want ack, then exchange happened, but we got info too
    nodeinfo.last_updated = hal->millis();
    bool needReply = (want_reply == true && !is_auto_full_node) && is_send_enabled;
    if (onNodeInfo) {
        onNodeInfo(header, nodeinfo, needReply);
    };
    if (want_reply && is_auto_full_node && is_send_enabled) {
        ESP_LOGI(TAG, "AUTO Sending my node info to node 0x%08" PRIx32, header.srcnode);
        SendMyNodeInfo(header.srcnode);
    }
}
void MeshtasticCompact::intOnWaypointMessage(MC_Header& header, MC_Waypoint& waypoint) {
    // should save this to waypoint database //todo
    if (onWaypointMessage) {
        onWaypointMessage(header, waypoint);
    };
}

void MeshtasticCompact::intOnTelemetryDevice(MC_Header& header, MC_Telemetry_Device& telemetry) {
    if (onTelemetryDevice) {
        onTelemetryDevice(header, telemetry);
    };
}

void MeshtasticCompact::intOnTelemetryEnvironment(MC_Header& header, MC_Telemetry_Environment& telemetry) {
    if (onTelemetryEnvironment) {
        onTelemetryEnvironment(header, telemetry);
    };
}

void MeshtasticCompact::intOnTraceroute(MC_Header& header, MC_RouteDiscovery& route_discovery) {
    // check is it for us
    if (header.dstnode == my_nodeinfo.node_id) {
        if (header.request_id == 0) {
            // we got a query
            // may add a callback here
            if (onTraceroute) {
                onTraceroute(header, route_discovery, true, false, (!(!is_in_stealth_mode && is_auto_full_node)) && is_send_enabled);
            }
            if (!is_in_stealth_mode && is_auto_full_node && is_send_enabled) {
                // send reply
                ESP_LOGE(TAG, "AUTO Sending traceroute reply to node 0x%08" PRIx32, header.srcnode);
                uint32_t tmp = header.srcnode;
                header.srcnode = my_nodeinfo.node_id;  // swap src and dst
                header.dstnode = tmp;
                header.hop_limit = header.hop_start;
                header.hop_start = send_hop_limit;
                header.request_id = header.packet_id;
                header.reply_id = header.packet_id;
                header.packet_id = 0;  // reset packet id for reply
                // add myself to the route
                if (route_discovery.snr_towards_count + 1 < 8) {
                    route_discovery.snr_towards_count++;
                    route_discovery.snr_towards[route_discovery.snr_towards_count - 1] = header.snr;
                }
                // send the reply
                SendTracerouteReply(header, route_discovery);
            }
        } else {
            // we got a reply
            // add our last snr to the end of the list
            if (route_discovery.snr_back_count + 1 < 8) {
                route_discovery.snr_back_count++;
                route_discovery.snr_back[route_discovery.snr_back_count - 1] = header.snr;
            }
            // add a callback
            if (onTraceroute) {
                onTraceroute(header, route_discovery, true, true, false);
            }
        }
        return;
    }
    // simply call callback
    if (onTraceroute) {
        onTraceroute(header, route_discovery, false, header.request_id == 0, (!(!is_in_stealth_mode && is_auto_full_node)) && is_send_enabled);
    }

    if (!is_in_stealth_mode && is_auto_full_node && is_send_enabled) {
        if (header.hop_limit > 0) {
            ESP_LOGI(TAG, "AUTO Flooding traceroute reply to node 0x%08" PRIx32, header.srcnode);
            // add myself to it
            if (header.request_id == 0) {
                if (route_discovery.route_count + 1 < 8) {
                    route_discovery.route_count++;
                    route_discovery.route[route_discovery.route_count - 1] = my_nodeinfo.node_id;
                    route_discovery.snr_towards_count++;
                    route_discovery.snr_towards[route_discovery.snr_towards_count - 1] = header.snr;
                }
            } else {
                if (route_discovery.route_back_count + 1 < 8) {
                    route_discovery.route_back_count++;
                    route_discovery.route_back[route_discovery.route_back_count - 1] = my_nodeinfo.node_id;
                    route_discovery.snr_back_count++;
                    route_discovery.snr_back[route_discovery.snr_back_count - 1] = header.snr;
                }
            }
            header.hop_limit--;  // Decrease hop limit
            SendTracerouteReply(header, route_discovery);
        }
    }
}

int16_t MeshtasticCompact::ProcessPacket(uint8_t* data, int len, MeshtasticCompact* mshcomp) {
    if (len > 0) {
        // https://meshtastic.org/docs/overview/mesh-algo/#layer-1-unreliable-zero-hop-messaging
        if (len < 0x10) {
            ESP_LOGE(TAG, "Received packet too short: %d bytes", len);
            return false;
        }
        MC_Header header;
        header.rssi = mshcomp->rssi;
        header.snr = mshcomp->snr;

        memcpy(&header.dstnode, &data[0], sizeof(uint32_t));
        memcpy(&header.srcnode, &data[4], sizeof(uint32_t));
        memcpy(&header.packet_id, &data[8], sizeof(uint32_t));

        if (!mshcomp->router.onCheck(header.srcnode, header.packet_id)) {
            return false;
        }
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
        int16_t ret = try_decode_root_packet(&data[16], len - 16, &meshtastic_Data_msg, &decodedtmp, sizeof(decodedtmp), header);
        if (ret >= 0) {
            // extract the want_response from bitfield
            decodedtmp.want_response |= decodedtmp.bitfield & BITFIELD_WANT_RESPONSE_MASK;
            ESP_LOGI(TAG, "PortNum: %d  PacketId: %lu  Src: %lu", decodedtmp.portnum, header.packet_id, header.srcnode);
            ESP_LOGI(TAG, "Want ack: %d", header.want_ack ? 1 : 0);
            ESP_LOGI(TAG, "Want Response: %d", decodedtmp.want_response);
            ESP_LOGI(TAG, "Request ID: %" PRIu32, decodedtmp.request_id);
            ESP_LOGI(TAG, "Reply ID: %" PRIu32, decodedtmp.reply_id);
            header.request_id = decodedtmp.request_id;
            header.reply_id = decodedtmp.reply_id;
            // Process the decoded data as needed https://github.com/meshtastic/protobufs/blob/master/meshtastic/portnums.proto
            if (decodedtmp.portnum == 0) {
                ESP_LOGI(TAG, "Received an unknown packet");
            } else if (decodedtmp.portnum == 1) {
                ESP_LOGI(TAG, "Received a message packet");
                // payload: utf8 text
                MC_TextMessage msg = {std::string(reinterpret_cast<const char*>(decodedtmp.payload.bytes), decodedtmp.payload.size), (uint8_t)ret, MC_MESSAGE_TYPE_TEXT};
                intOnMessage(header, msg);
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
                // payload: protobuf Position
                meshtastic_Position position_msg = {};
                if (pb_decode_from_bytes(decodedtmp.payload.bytes, decodedtmp.payload.size, &meshtastic_Position_msg, &position_msg)) {
                    MC_Position position = {.latitude_i = position_msg.latitude_i, .longitude_i = position_msg.longitude_i, .altitude = position_msg.altitude, .ground_speed = position_msg.ground_speed, .sats_in_view = position_msg.sats_in_view, .location_source = (uint8_t)position_msg.location_source, .has_latitude_i = position_msg.has_latitude_i, .has_longitude_i = position_msg.has_longitude_i, .has_altitude = position_msg.has_altitude, .has_ground_speed = position_msg.has_ground_speed};
                    intOnPositionMessage(header, position, decodedtmp.want_response);

                } else {
                    ESP_LOGE(TAG, "Failed to decode Position");
                }
            } else if (decodedtmp.portnum == 4) {
                // payload: protobuf User
                meshtastic_User user_msg = {};
                if (pb_decode_from_bytes(decodedtmp.payload.bytes, decodedtmp.payload.size, &meshtastic_User_msg, &user_msg)) {
                    MC_NodeInfo node_info;
                    node_info.node_id = header.srcnode;  // srcnode is the node ID
                    memcpy(node_info.id, user_msg.id, sizeof(node_info.id));
                    memcpy(node_info.short_name, user_msg.short_name, sizeof(node_info.short_name));
                    memcpy(node_info.long_name, user_msg.long_name, sizeof(node_info.long_name));
                    memcpy(node_info.macaddr, user_msg.macaddr, sizeof(node_info.macaddr));
                    memcpy(node_info.public_key, user_msg.public_key.bytes, sizeof(node_info.public_key));
                    node_info.public_key_size = user_msg.public_key.size;
                    node_info.role = user_msg.role;
                    node_info.hw_model = user_msg.hw_model;
                    intOnNodeInfo(header, node_info, decodedtmp.want_response);
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
                MC_TextMessage msg = {std::string(reinterpret_cast<const char*>(uncompressed_data), uncompressed_size), (uint8_t)ret, MC_MESSAGE_TYPE_TEXT};
                intOnMessage(header, msg);
            } else if (decodedtmp.portnum == 8) {
                ESP_LOGI(TAG, "Received a waypoint packet");
                // payload: protobuf Waypoint
                meshtastic_Waypoint waypoint_msg = {};  // todo store and callbacke
                if (pb_decode_from_bytes(decodedtmp.payload.bytes, decodedtmp.payload.size, &meshtastic_Waypoint_msg, &waypoint_msg)) {
                    MC_Waypoint waypoint;
                    waypoint.latitude_i = waypoint_msg.latitude_i;
                    waypoint.longitude_i = waypoint_msg.longitude_i;
                    memcpy(waypoint.name, waypoint_msg.name, sizeof(waypoint.name));
                    memcpy(waypoint.description, waypoint_msg.description, sizeof(waypoint.description));
                    waypoint.icon = waypoint_msg.icon;
                    waypoint.expire = waypoint_msg.expire;
                    waypoint.id = waypoint_msg.id;
                    waypoint.has_latitude_i = waypoint_msg.has_latitude_i;
                    waypoint.has_longitude_i = waypoint_msg.has_longitude_i;
                    intOnWaypointMessage(header, waypoint);
                } else {
                    ESP_LOGE(TAG, "Failed to decode Waypoint");
                }
            } else if (decodedtmp.portnum == 10) {
                ESP_LOGI(TAG, "Received a detection sensor packet");
                // payload: utf8 text
                MC_TextMessage msg = {std::string(reinterpret_cast<const char*>(decodedtmp.payload.bytes), decodedtmp.payload.size), (uint8_t)ret, MC_MESSAGE_TYPE_DETECTOR_SENSOR};
                intOnMessage(header, msg);
            } else if (decodedtmp.portnum == 11) {
                ESP_LOGI(TAG, "Received an alert packet");
                // payload: utf8 text
                MC_TextMessage msg = {std::string(reinterpret_cast<const char*>(decodedtmp.payload.bytes), decodedtmp.payload.size), (uint8_t)ret, MC_MESSAGE_TYPE_ALERT};
                intOnMessage(header, msg);
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
                MC_TextMessage msg = {std::string(reinterpret_cast<const char*>(decodedtmp.payload.bytes), decodedtmp.payload.size), (uint8_t)ret, MC_MESSAGE_TYPE_PING};
                intOnMessage(header, msg);
            } else if (decodedtmp.portnum == 34) {
                ESP_LOGI(TAG, "Received a paxcounter packet");
                // payload: protobuf DROP
            } else if (decodedtmp.portnum == 64) {
                ESP_LOGI(TAG, "Received a serial packet");
                // payload: uart rx/tx data
                MC_TextMessage msg = {std::string(reinterpret_cast<const char*>(decodedtmp.payload.bytes), decodedtmp.payload.size), (uint8_t)ret, MC_MESSAGE_TYPE_UART};
                intOnMessage(header, msg);
            } else if (decodedtmp.portnum == 65) {
                ESP_LOGI(TAG, "Received a STORE_FORWARD_APP  packet");
                // payload: ?
            } else if (decodedtmp.portnum == 66) {
                ESP_LOGI(TAG, "Received a RANGE_TEST_APP  packet");
                // payload: ascii text
                MC_TextMessage msg = {std::string(reinterpret_cast<const char*>(decodedtmp.payload.bytes), decodedtmp.payload.size), (uint8_t)ret, MC_MESSAGE_TYPE_RANGE_TEST};
                intOnMessage(header, msg);
            } else if (decodedtmp.portnum == 67) {
                ESP_LOGI(TAG, "Received a TELEMETRY_APP   packet");
                // payload: Protobuf
                meshtastic_Telemetry telemetry_msg = {};  // todo store and callback
                if (pb_decode_from_bytes(decodedtmp.payload.bytes, decodedtmp.payload.size, &meshtastic_Telemetry_msg, &telemetry_msg)) {
                    // ESP_LOGI(TAG, "Telemetry Time: %lu", telemetry_msg.time);
                    switch (telemetry_msg.which_variant) {
                        case meshtastic_Telemetry_device_metrics_tag:
                            MC_Telemetry_Device device_metrics;
                            device_metrics.battery_level = telemetry_msg.variant.device_metrics.battery_level;
                            device_metrics.uptime_seconds = telemetry_msg.variant.device_metrics.uptime_seconds;
                            device_metrics.voltage = telemetry_msg.variant.device_metrics.voltage;
                            device_metrics.channel_utilization = telemetry_msg.variant.device_metrics.channel_utilization;
                            device_metrics.has_battery_level = telemetry_msg.variant.device_metrics.has_battery_level;
                            device_metrics.has_uptime_seconds = telemetry_msg.variant.device_metrics.has_uptime_seconds;
                            device_metrics.has_voltage = telemetry_msg.variant.device_metrics.has_voltage;
                            device_metrics.has_channel_utilization = telemetry_msg.variant.device_metrics.has_channel_utilization;
                            intOnTelemetryDevice(header, device_metrics);
                            break;
                        case meshtastic_Telemetry_environment_metrics_tag:
                            MC_Telemetry_Environment environment_metrics;
                            environment_metrics.temperature = telemetry_msg.variant.environment_metrics.temperature;
                            environment_metrics.humidity = telemetry_msg.variant.environment_metrics.relative_humidity;
                            environment_metrics.pressure = telemetry_msg.variant.environment_metrics.barometric_pressure;
                            environment_metrics.lux = telemetry_msg.variant.environment_metrics.lux;
                            environment_metrics.has_temperature = telemetry_msg.variant.environment_metrics.has_temperature;
                            environment_metrics.has_humidity = telemetry_msg.variant.environment_metrics.has_relative_humidity;
                            environment_metrics.has_pressure = telemetry_msg.variant.environment_metrics.has_barometric_pressure;
                            environment_metrics.has_lux = telemetry_msg.variant.environment_metrics.has_lux;
                            intOnTelemetryEnvironment(header, environment_metrics);
                            break;
                        case meshtastic_Telemetry_air_quality_metrics_tag:
                            // ESP_LOGI(TAG, "Air Quality Metrics: PM2.5: %lu ", telemetry_msg.variant.air_quality_metrics.pm25_standard);
                            // skipping, not interesting yet PR-s are welcome
                            break;
                        case meshtastic_Telemetry_power_metrics_tag:
                            // skipping, not interesting yet PR-s are welcome
                            break;
                        case meshtastic_Telemetry_local_stats_tag:
                            // skipping, not interesting yet PR-s are welcome
                            break;
                        case meshtastic_Telemetry_health_metrics_tag:
                            // ESP_LOGI(TAG, "Health Metrics: Hearth BPM: %u, Temp: %f  So2: %u", telemetry_msg.variant.health_metrics.heart_bpm, telemetry_msg.variant.health_metrics.temperature, telemetry_msg.variant.health_metrics.spO2);
                            //  skipping, not interesting yet PR-s are welcome
                            break;
                        case meshtastic_Telemetry_host_metrics_tag:
                            // skipping, not interesting yet PR-s are welcome
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
                    // header.request_id ==0 --route back
                    MC_RouteDiscovery route_discovery;
                    route_discovery.route_count = route_discovery_msg.route_count;
                    route_discovery.snr_towards_count = route_discovery_msg.snr_towards_count;
                    route_discovery.route_back_count = route_discovery_msg.route_back_count;
                    route_discovery.snr_back_count = route_discovery_msg.snr_back_count;
                    memcpy(route_discovery.route, route_discovery_msg.route, sizeof(route_discovery.route));
                    memcpy(route_discovery.snr_towards, route_discovery_msg.snr_towards, sizeof(route_discovery.snr_towards));
                    memcpy(route_discovery.route_back, route_discovery_msg.route_back, sizeof(route_discovery.route_back));
                    memcpy(route_discovery.snr_back, route_discovery_msg.snr_back, sizeof(route_discovery.snr_back));
                    intOnTraceroute(header, route_discovery);
                } else {
                    ESP_LOGE(TAG, "Failed to decode RouteDiscovery");
                }
            } else if (decodedtmp.portnum == 71) {
                ESP_LOGI(TAG, "Received a NEIGHBORINFO_APP   packet");
                // payload: Protobuf ?
            } else {
                ESP_LOGI(TAG, "Received an unhandled portnum: %d", decodedtmp.portnum);
            }
            if (header.want_ack && is_send_enabled && !is_in_stealth_mode && header.dstnode == my_nodeinfo.node_id) {
                // send_ack(header);
            }
        }
        return ret;
    }
    return false;
}

void MeshtasticCompact::setMyNames(const char* short_name, const char* long_name) {
    strncpy(my_nodeinfo.short_name, short_name, sizeof(my_nodeinfo.short_name) - 1);
    my_nodeinfo.short_name[sizeof(my_nodeinfo.short_name) - 1] = '\0';
    strncpy(my_nodeinfo.long_name, long_name, sizeof(my_nodeinfo.long_name) - 1);
    my_nodeinfo.long_name[sizeof(my_nodeinfo.long_name) - 1] = '\0';
}

#pragma region Decoder Helpers

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
        // printf("Panic: can't encode protobuf reason='%s'", PB_GET_ERROR(&stream));
        return 0;
    } else {
        return stream.bytes_written;
    }
}

int16_t MeshtasticCompact::try_decode_root_packet(const uint8_t* srcbuf, size_t srcbufsize, const pb_msgdesc_t* fields, void* dest_struct, size_t dest_struct_size, MC_Header& header) {
    uint8_t decrypted_data[srcbufsize] = {0};
    memset(dest_struct, 0, dest_struct_size);
    // 1st.
    if (aes_decrypt_meshtastic_payload(default_l1_key, sizeof(default_l1_key) * 8, header.packet_id, header.srcnode, srcbuf, decrypted_data, srcbufsize)) {
        if (pb_decode_from_bytes(decrypted_data, srcbufsize, fields, dest_struct)) return 254;
    }
    memset(dest_struct, 0, dest_struct_size);
    if (aes_decrypt_meshtastic_payload(default_chan_key, sizeof(default_chan_key) * 8, header.packet_id, header.srcnode, srcbuf, decrypted_data, srcbufsize)) {
        if (pb_decode_from_bytes(decrypted_data, srcbufsize, fields, dest_struct)) return 0;
    }

    if (header.chan_hash == 0 && header.dstnode != 0xffffffff) {
        // todo pki decrypt
        ESP_LOGI(TAG, "can't decode priv packet");
        return -1;
    }
    // todo iterate chan keys

    ESP_LOGI(TAG, "can't decode packet");
    return -1;
}

#pragma endregion

#pragma region PacketBuilders

void MeshtasticCompact::send_ack(MC_Header& header) {
    if (!is_send_enabled) return;
    if (is_in_stealth_mode) return;
    MC_OutQueueEntry entry;
    entry.header.dstnode = header.srcnode;  // Send ACK to the source node
    entry.header.srcnode = my_nodeinfo.node_id;
    entry.header.packet_id = header.packet_id;  // Use the same packet ID for ACK
    entry.header.hop_limit = send_hop_limit;
    entry.header.want_ack = 0;
    entry.header.via_mqtt = false;
    entry.data.request_id = header.request_id;
    entry.header.chan_hash = header.chan_hash;
    entry.encType = 1;
    entry.data.portnum = meshtastic_PortNum_ROUTING_APP;
    entry.data.want_response = 0;
    entry.key = (uint8_t*)default_l1_key;
    entry.key_len = sizeof(default_l1_key);
    meshtastic_Routing c = meshtastic_Routing_init_default;
    c.error_reason = meshtastic_Routing_Error_NONE;  // No error reason for ACK
    c.which_variant = meshtastic_Routing_error_reason_tag;
    entry.data.payload.size = pb_encode_to_bytes(entry.data.payload.bytes, sizeof(entry.data.payload.bytes), &meshtastic_Routing_msg, &c);
    out_queue.push(entry);
}

void MeshtasticCompact::SendTracerouteReply(MC_Header& header, MC_RouteDiscovery& route_discovery) {
    if (!is_send_enabled) return;
    if (is_in_stealth_mode) return;

    MC_OutQueueEntry entry;
    entry.header.dstnode = header.dstnode;
    entry.header.srcnode = header.srcnode;
    entry.header.packet_id = header.packet_id;
    entry.header.hop_limit = header.hop_limit;
    entry.header.want_ack = 0;
    entry.header.via_mqtt = false;
    entry.header.hop_start = header.hop_start;
    entry.header.chan_hash = header.chan_hash;
    entry.header.via_mqtt = 0;
    entry.header.reply_id = header.reply_id;
    entry.header.request_id = header.request_id;
    entry.encType = 1;
    entry.data.request_id = header.request_id;
    entry.data.reply_id = 0;
    entry.data.portnum = meshtastic_PortNum_TRACEROUTE_APP;
    entry.data.want_response = 0;  // this is reply, so no need for response
    entry.data.bitfield = 1;
    entry.data.has_bitfield = true;
    entry.data.dest = 0;
    entry.data.source = 0;
    entry.data.emoji = 0;
    meshtastic_RouteDiscovery meshtastic_route_discovery = meshtastic_RouteDiscovery_init_default;
    meshtastic_route_discovery.route_count = route_discovery.route_count;
    meshtastic_route_discovery.snr_towards_count = route_discovery.snr_towards_count;
    meshtastic_route_discovery.route_back_count = route_discovery.route_back_count;
    meshtastic_route_discovery.snr_back_count = route_discovery.snr_back_count;
    memcpy(meshtastic_route_discovery.route, route_discovery.route, sizeof(meshtastic_route_discovery.route));
    memcpy(meshtastic_route_discovery.snr_towards, route_discovery.snr_towards, sizeof(meshtastic_route_discovery.snr_towards));
    memcpy(meshtastic_route_discovery.route_back, route_discovery.route_back, sizeof(meshtastic_route_discovery.route_back));
    memcpy(meshtastic_route_discovery.snr_back, route_discovery.snr_back, sizeof(meshtastic_route_discovery.snr_back));
    entry.data.payload.size = pb_encode_to_bytes(entry.data.payload.bytes, sizeof(entry.data.payload.bytes), &meshtastic_RouteDiscovery_msg, &meshtastic_route_discovery);
    entry.key = (uint8_t*)default_l1_key;
    entry.key_len = sizeof(default_l1_key);
    out_queue.push(entry);
}

void MeshtasticCompact::SendTraceroute(uint32_t dest_node_id, uint8_t chan, uint32_t sender_node_id) {
    if (!is_send_enabled) return;
    if (is_in_stealth_mode) return;

    MC_OutQueueEntry entry;
    entry.header.dstnode = dest_node_id;
    entry.header.srcnode = sender_node_id == 0 ? my_nodeinfo.node_id : sender_node_id;
    entry.header.packet_id = 0;
    entry.header.hop_limit = send_hop_limit;
    entry.header.want_ack = 1;
    entry.header.via_mqtt = false;
    entry.header.hop_start = send_hop_limit;
    entry.header.chan_hash = chan;
    entry.header.via_mqtt = 0;
    entry.encType = 1;
    meshtastic_RouteDiscovery route_discovery_msg = meshtastic_RouteDiscovery_init_default;
    entry.data.portnum = meshtastic_PortNum_TRACEROUTE_APP;
    entry.data.want_response = true;
    entry.data.bitfield = 0;
    entry.data.payload.size = pb_encode_to_bytes(entry.data.payload.bytes, sizeof(entry.data.payload.bytes), &meshtastic_RouteDiscovery_msg, &route_discovery_msg);
    entry.key = (uint8_t*)default_l1_key;    // Use default channel key for encryption
    entry.key_len = sizeof(default_l1_key);  // Use default channel key length
    out_queue.push(entry);
}

void MeshtasticCompact::SendNodeInfo(MC_NodeInfo& nodeinfo, uint32_t dstnode, bool exchange) {
    if (!is_send_enabled) return;
    MC_OutQueueEntry entry;
    entry.header.dstnode = dstnode;
    entry.header.srcnode = nodeinfo.node_id;
    entry.header.packet_id = 0;
    entry.header.hop_limit = send_hop_limit;
    entry.header.want_ack = 0;
    entry.header.via_mqtt = false;
    entry.header.hop_start = send_hop_limit;
    entry.header.chan_hash = 8;  // Use default channel hash
    entry.header.via_mqtt = 0;   // Not used in this case
    entry.encType = 1;           // AES encryption
    meshtastic_User user_msg = {};
    memcpy(user_msg.id, nodeinfo.id, sizeof(user_msg.id));
    memcpy(user_msg.short_name, nodeinfo.short_name, sizeof(user_msg.short_name));
    memcpy(user_msg.long_name, nodeinfo.long_name, sizeof(user_msg.long_name));
    memcpy(user_msg.macaddr, nodeinfo.macaddr, sizeof(user_msg.macaddr));
    memcpy(user_msg.public_key.bytes, nodeinfo.public_key, sizeof(user_msg.public_key.bytes));
    user_msg.public_key.size = nodeinfo.public_key_size;
    entry.data.bitfield = 0;  // todo check
    bool all_zero = true;
    for (size_t i = 0; i < sizeof(user_msg.public_key.bytes); i++) {
        if (user_msg.public_key.bytes[i] != 0) {
            all_zero = false;
            break;
        }
    }
    if (all_zero) {
        user_msg.public_key.size = 0;  // Set size to 0 if all bytes are zero //todo maybe delete
    }
    user_msg.role = (meshtastic_Config_DeviceConfig_Role)nodeinfo.role;
    user_msg.hw_model = (meshtastic_HardwareModel)nodeinfo.hw_model;
    user_msg.is_licensed = false;
    user_msg.is_unmessagable = false;
    entry.data.portnum = meshtastic_PortNum_NODEINFO_APP;
    entry.data.want_response = exchange;
    entry.data.payload.size = pb_encode_to_bytes((uint8_t*)&entry.data.payload.bytes, sizeof(entry.data.payload.bytes), &meshtastic_User_msg, &user_msg);
    entry.key = (uint8_t*)default_l1_key;    // Use default channel key for encryption
    entry.key_len = sizeof(default_l1_key);  // Use default channel key length
    out_queue.push(entry);
}

void MeshtasticCompact::SendTextMessage(const std::string& text, uint32_t dstnode, uint8_t chan, MC_MESSAGE_TYPE type, uint32_t sender_node_id) {
    if (!is_send_enabled) return;
    MC_OutQueueEntry entry;
    entry.header.dstnode = dstnode;
    entry.header.srcnode = sender_node_id == 0 ? my_nodeinfo.node_id : sender_node_id;
    entry.header.packet_id = 0;
    entry.header.hop_limit = send_hop_limit;
    entry.header.want_ack = 1;
    entry.header.via_mqtt = false;
    entry.header.hop_start = send_hop_limit;
    entry.header.chan_hash = chan;
    entry.header.via_mqtt = 0;
    entry.encType = 1;
    entry.data.payload.size = text.size();
    memcpy(entry.data.payload.bytes, text.data(), text.size());
    entry.data.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;  // NodeInfo portnum
    entry.data.want_response = 0;
    entry.data.bitfield = 0;
    entry.key = (uint8_t*)default_l1_key;
    entry.key_len = sizeof(default_l1_key);  // Use default channel key for encryption
    out_queue.push(entry);
}

void MeshtasticCompact::SendRequestPositionInfo(uint32_t dest_node_id, uint8_t chan, uint32_t sender_node_id) {
    if (!is_send_enabled) return;  // todo check if this works or deletes the position
    MC_OutQueueEntry entry;
    entry.header.dstnode = dest_node_id;
    entry.header.srcnode = sender_node_id == 0 ? my_nodeinfo.node_id : sender_node_id;
    entry.header.packet_id = 0;
    entry.header.hop_limit = send_hop_limit;
    entry.header.want_ack = 0;
    entry.header.via_mqtt = false;
    entry.header.hop_start = send_hop_limit;
    entry.header.chan_hash = chan;
    entry.header.via_mqtt = 0;
    entry.encType = 1;
    entry.data.portnum = meshtastic_PortNum_POSITION_APP;
    entry.data.want_response = 1;
    entry.data.bitfield = 0;
    entry.key = (uint8_t*)default_l1_key;
    entry.key_len = sizeof(default_l1_key);  // Use default channel key for encryption
    meshtastic_Position position_msg = {};
    entry.data.payload.size = pb_encode_to_bytes((uint8_t*)&entry.data.payload.bytes, sizeof(entry.data.payload.bytes), &meshtastic_Position_msg, &position_msg);
    out_queue.push(entry);
}

void MeshtasticCompact::SendPositionMessage(MC_Position& position, uint32_t dstnode, uint8_t chan, uint32_t sender_node_id) {
    if (!is_send_enabled) return;
    MC_OutQueueEntry entry;
    entry.header.dstnode = dstnode;
    entry.header.srcnode = sender_node_id == 0 ? my_nodeinfo.node_id : sender_node_id;
    entry.header.packet_id = 0;
    entry.header.hop_limit = send_hop_limit;
    entry.header.want_ack = 0;
    entry.header.via_mqtt = false;
    entry.header.hop_start = send_hop_limit;
    entry.header.chan_hash = chan;
    entry.header.via_mqtt = 0;
    entry.encType = 1;
    entry.data.portnum = meshtastic_PortNum_POSITION_APP;
    entry.data.bitfield = 0;
    entry.data.want_response = 0;
    entry.key = (uint8_t*)default_l1_key;
    entry.key_len = sizeof(default_l1_key);  // Use default channel key for encryption
    meshtastic_Position position_msg = {};

    position_msg.latitude_i = position.latitude_i;
    position_msg.longitude_i = position.longitude_i;
    position_msg.altitude = position.altitude;
    position_msg.ground_speed = position.ground_speed;
    position_msg.sats_in_view = position.sats_in_view;
    position_msg.location_source = (meshtastic_Position_LocSource)position.location_source;
    position_msg.precision_bits = 17;
    position_msg.has_altitude = position.altitude != 0;
    position_msg.has_ground_speed = position.ground_speed != 0;
    position_msg.has_latitude_i = true;
    position_msg.has_longitude_i = true;
    position_msg.has_altitude = position.altitude != 0;
    entry.data.payload.size = pb_encode_to_bytes((uint8_t*)&entry.data.payload.bytes, sizeof(entry.data.payload.bytes), &meshtastic_Position_msg, &position_msg);
    out_queue.push(entry);
}

void MeshtasticCompact::SendWaypointMessage(MC_Waypoint& waypoint, uint32_t dstnode, uint8_t chan, uint32_t sender_node_id) {
    if (!is_send_enabled) return;
    MC_OutQueueEntry entry;
    entry.header.dstnode = dstnode;
    entry.header.srcnode = sender_node_id == 0 ? my_nodeinfo.node_id : sender_node_id;
    entry.header.packet_id = 0;
    entry.header.hop_limit = send_hop_limit;
    entry.header.want_ack = 0;
    entry.header.via_mqtt = false;
    entry.header.hop_start = send_hop_limit;
    entry.header.chan_hash = chan;
    entry.header.via_mqtt = 0;
    entry.encType = 1;  // AES encryption
    entry.data.portnum = meshtastic_PortNum_WAYPOINT_APP;
    entry.data.want_response = 0;
    entry.data.bitfield = 0;
    entry.key = (uint8_t*)default_l1_key;
    entry.key_len = sizeof(default_l1_key);  // Use default channel key for encryption
    meshtastic_Waypoint waypoint_msg = {};

    waypoint_msg.latitude_i = waypoint.latitude_i;
    waypoint_msg.longitude_i = waypoint.longitude_i;
    memcpy(waypoint_msg.name, waypoint.name, sizeof(waypoint_msg.name));
    memcpy(waypoint_msg.description, waypoint.description, sizeof(waypoint_msg.description));
    waypoint_msg.icon = waypoint.icon;
    waypoint_msg.expire = waypoint.expire;
    waypoint_msg.id = waypoint.id;
    entry.data.payload.size = pb_encode_to_bytes((uint8_t*)&entry.data.payload.bytes, sizeof(entry.data.payload.bytes), &meshtastic_Waypoint_msg, &waypoint_msg);
    out_queue.push(entry);
}

void MeshtasticCompact::SendTelemetryDevice(MC_Telemetry_Device& telemetry, uint32_t dstnode, uint8_t chan, uint32_t sender_node_id) {
    if (!is_send_enabled) return;
    MC_OutQueueEntry entry;
    entry.header.dstnode = dstnode;
    entry.header.srcnode = sender_node_id == 0 ? my_nodeinfo.node_id : sender_node_id;
    entry.header.packet_id = 0;
    entry.header.hop_limit = send_hop_limit;
    entry.header.want_ack = 0;
    entry.header.via_mqtt = false;
    entry.header.hop_start = send_hop_limit;
    entry.header.chan_hash = chan;
    entry.header.via_mqtt = 0;
    entry.encType = 1;
    entry.data.portnum = meshtastic_PortNum_TELEMETRY_APP;
    entry.data.want_response = 0;
    entry.key = (uint8_t*)default_l1_key;
    entry.key_len = sizeof(default_l1_key);
    meshtastic_Telemetry telemetry_msg = {};

    telemetry_msg.time = (uint32_t)time(NULL);
    telemetry_msg.which_variant = meshtastic_Telemetry_device_metrics_tag;
    telemetry_msg.variant.device_metrics.battery_level = telemetry.battery_level;
    telemetry_msg.variant.device_metrics.uptime_seconds = telemetry.uptime_seconds;
    telemetry_msg.variant.device_metrics.voltage = telemetry.voltage;
    telemetry_msg.variant.device_metrics.channel_utilization = telemetry.channel_utilization;
    telemetry_msg.variant.device_metrics.has_battery_level = telemetry.has_battery_level;
    telemetry_msg.variant.device_metrics.has_uptime_seconds = telemetry.has_uptime_seconds;
    telemetry_msg.variant.device_metrics.has_voltage = telemetry.has_voltage;
    telemetry_msg.variant.device_metrics.has_channel_utilization = telemetry.has_channel_utilization;
    entry.data.bitfield = 0;
    entry.data.payload.size = pb_encode_to_bytes((uint8_t*)&entry.data.payload.bytes, sizeof(entry.data.payload.bytes), &meshtastic_Telemetry_msg, &telemetry_msg);
    out_queue.push(entry);
}

void MeshtasticCompact::SendTelemetryEnvironment(MC_Telemetry_Environment& telemetry, uint32_t dstnode, uint8_t chan, uint32_t sender_node_id) {
    if (!is_send_enabled) return;
    MC_OutQueueEntry entry;
    entry.header.dstnode = dstnode;
    entry.header.srcnode = sender_node_id == 0 ? my_nodeinfo.node_id : sender_node_id;
    entry.header.packet_id = 0;
    entry.header.hop_limit = send_hop_limit;
    entry.header.want_ack = 0;
    entry.header.via_mqtt = false;
    entry.header.hop_start = send_hop_limit;
    entry.header.chan_hash = chan;
    entry.header.via_mqtt = 0;
    entry.encType = 1;
    entry.data.portnum = meshtastic_PortNum_TELEMETRY_APP;
    entry.data.want_response = 0;
    entry.key = (uint8_t*)default_l1_key;
    entry.key_len = sizeof(default_l1_key);
    meshtastic_Telemetry telemetry_msg = {};

    telemetry_msg.time = (uint32_t)time(NULL);
    telemetry_msg.which_variant = meshtastic_Telemetry_environment_metrics_tag;
    telemetry_msg.variant.environment_metrics.temperature = telemetry.temperature;
    telemetry_msg.variant.environment_metrics.has_temperature = telemetry.has_temperature;
    telemetry_msg.variant.environment_metrics.relative_humidity = telemetry.humidity;
    telemetry_msg.variant.environment_metrics.has_relative_humidity = telemetry.has_humidity;
    telemetry_msg.variant.environment_metrics.barometric_pressure = telemetry.pressure;
    telemetry_msg.variant.environment_metrics.has_barometric_pressure = telemetry.has_pressure;
    telemetry_msg.variant.environment_metrics.lux = telemetry.lux;
    telemetry_msg.variant.environment_metrics.has_lux = telemetry.has_lux;
    entry.data.bitfield = 0;
    entry.data.payload.size = pb_encode_to_bytes((uint8_t*)&entry.data.payload.bytes, sizeof(entry.data.payload.bytes), &meshtastic_Telemetry_msg, &telemetry_msg);
    out_queue.push(entry);
}

inline uint8_t MeshtasticCompact::getLastByteOfNodeNum(uint32_t num) {
    return (uint8_t)((num & 0xFF) ? (num & 0xFF) : 0xFF);
}

#pragma endregion

#pragma region MeshtasticCompactHelpers

void MeshtasticCompactHelpers::NodeInfoBuilder(MC_NodeInfo& nodeinfo, uint32_t node_id, std::string& short_name, std::string& long_name) {
    nodeinfo.node_id = node_id;
    if (long_name.empty()) {
        char hex_part[7];
        snprintf(hex_part, sizeof(hex_part), "%06" PRIx32, (node_id & 0xFFFFFF));
        hex_part[sizeof(hex_part) - 1] = '\0';
        std::string generated_name = "Meshtastic-" + std::string(hex_part);
        long_name = generated_name;
    }
    if (short_name.empty()) {
        char hex_part[5];
        snprintf(hex_part, sizeof(hex_part), "%04" PRIx32, node_id & 0xFFFF);
        hex_part[sizeof(hex_part) - 1] = '\0';
        short_name = std::string(hex_part);
    }

    strncpy(nodeinfo.short_name, short_name.c_str(), sizeof(nodeinfo.short_name) - 1);
    nodeinfo.short_name[sizeof(nodeinfo.short_name) - 1] = '\0';
    strncpy(nodeinfo.long_name, long_name.c_str(), sizeof(nodeinfo.long_name) - 1);
    nodeinfo.long_name[sizeof(nodeinfo.long_name) - 1] = '\0';
    nodeinfo.hw_model = (uint8_t)meshtastic_HardwareModel_DIY_V1;
    snprintf(nodeinfo.id, sizeof(nodeinfo.id), "!%08" PRIx32, node_id);
    nodeinfo.id[sizeof(nodeinfo.id) - 1] = '0';
    nodeinfo.role = 0;
    for (int i = 0; i < 6; ++i) {
        nodeinfo.macaddr[i] = (node_id >> (8 * (5 - i))) & 0xFF;
    }
    memset(nodeinfo.public_key, 0, sizeof(nodeinfo.public_key));
    nodeinfo.public_key_size = 0;  // Set to 0 if no public key is available
}

void MeshtasticCompactHelpers::PositionBuilder(MC_Position& position, float latitude, float longitude, int32_t altitude, uint32_t speed, uint32_t sats_in_view) {
    position.latitude_i = static_cast<int32_t>(latitude * 10e6);
    position.longitude_i = static_cast<int32_t>(longitude * 10e6);
    position.altitude = altitude;
    position.ground_speed = speed;
    position.sats_in_view = sats_in_view;
    position.location_source = 0;
}

void MeshtasticCompactHelpers::TelemetryDeviceBuilder(MC_Telemetry_Device& telemetry, uint32_t uptime_seconds, float voltage, float battery_level, float channel_utilization) {
    telemetry.uptime_seconds = uptime_seconds;
    telemetry.voltage = voltage;
    telemetry.battery_level = battery_level;
    telemetry.channel_utilization = channel_utilization;
    telemetry.has_uptime_seconds = (uptime_seconds != 0);
    telemetry.has_voltage = (voltage >= 0.0f);
    telemetry.has_battery_level = (battery_level >= 0.0f);
    telemetry.has_channel_utilization = (channel_utilization >= 0.0f);
}

void MeshtasticCompactHelpers::TelemetryEnvironmentBuilder(MC_Telemetry_Environment& telemetry, float temperature, float humidity, float pressure, float lux) {
    telemetry.temperature = temperature;
    telemetry.humidity = humidity;
    telemetry.pressure = pressure;
    telemetry.lux = lux;
    telemetry.has_temperature = (temperature > -10000.0f);
    telemetry.has_humidity = (humidity >= 0.0f);
    telemetry.has_pressure = (pressure >= 0.0f);
    telemetry.has_lux = (lux >= 0.0f);
}

void MeshtasticCompactHelpers::WaypointBuilder(MC_Waypoint& waypoint, uint32_t id, float latitude, float longitude, std::string name, std::string description, uint32_t expire, uint32_t icon) {
    waypoint.latitude_i = static_cast<int32_t>(latitude * 10e6);
    waypoint.longitude_i = static_cast<int32_t>(longitude * 10e6);
    strncpy(waypoint.name, name.c_str(), sizeof(waypoint.name) - 1);
    waypoint.name[sizeof(waypoint.name) - 1] = '\0';
    strncpy(waypoint.description, description.c_str(), sizeof(waypoint.description) - 1);
    waypoint.description[sizeof(waypoint.description) - 1] = '\0';
    waypoint.icon = icon;
    waypoint.expire = expire;
    waypoint.id = id;
    waypoint.has_latitude_i = waypoint.latitude_i != 0;
    waypoint.has_longitude_i = waypoint.longitude_i != 0;
}
#pragma endregion