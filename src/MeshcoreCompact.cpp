#include "MeshcoreCompact.hpp"

#include "esp_log.h"
#include "esp_mac.h"
#define TAG "MeshcoreCompact"

volatile bool packetFlag = false;

static_assert(CONFIG_ESP_MAIN_TASK_STACK_SIZE >= 8000, "Main task stack size must be at least 8000 bytes!");

void IRAM_ATTR onPacketReceived() {
    packetFlag = true;
}

MeshcoreCompact::MeshcoreCompact() {
}

MeshcoreCompact::~MeshcoreCompact() {
    need_run = false;  // Stop the tasks
    packetFlag = true;
    // out_queue.stop_wait();                 // Notify any waiting pop() to unblock immediately
    vTaskDelay(150 / portTICK_PERIOD_MS);  // Give some time for tasks to finish
    if (radio) {
        radio->standby();
        delete radio;  // Delete the radio instance
        radio = nullptr;
    }
    if (hal) {
        hal->term();
        delete (EspHal*)hal;
        hal = nullptr;
    }
}

bool MeshcoreCompact::setRadioFrequency(float freq) {
    if (radio == nullptr) {
        return false;
    }
    int state = RADIOLIB_ERR_NONE;
    {
        std::lock_guard<std::mutex> lock(mtx_radio);
        state = radio->setFrequency(freq);
    }
    return (state == RADIOLIB_ERR_NONE);
}
bool MeshcoreCompact::setRadioSpreadingFactor(uint8_t sf) {
    if (radio == nullptr) {
        return false;
    }
    int state = RADIOLIB_ERR_NONE;
    {
        std::lock_guard<std::mutex> lock(mtx_radio);
        switch (radio_type) {
            case RadioType::SX1261:
            case RadioType::SX1262:
            case RadioType::SX1268:
                state = ((SX126x*)radio)->setSpreadingFactor(sf);
                break;
            case RadioType::SX1276:
                state = ((SX1276*)radio)->setSpreadingFactor(sf);
                break;
            default:
                state = RADIOLIB_ERR_UNKNOWN;
                break;
        }
        return (state == RADIOLIB_ERR_NONE);
    }
}

bool MeshcoreCompact::setRadioBandwidth(uint32_t bw) {
    if (radio == nullptr) {
        return false;
    }
    int state = RADIOLIB_ERR_NONE;
    {
        std::lock_guard<std::mutex> lock(mtx_radio);
        switch (radio_type) {
            case RadioType::SX1261:
            case RadioType::SX1262:
            case RadioType::SX1268:
                state = ((SX126x*)radio)->setBandwidth(bw);
                break;
            case RadioType::SX1276:
                state = ((SX1276*)radio)->setBandwidth(bw);
                break;
            default:
                state = RADIOLIB_ERR_UNKNOWN;
                break;
        }
        return (state == RADIOLIB_ERR_NONE);
    }
}
bool MeshcoreCompact::setRadioCodingRate(uint8_t cr) {
    if (radio == nullptr) {
        return false;
    }
    int state = RADIOLIB_ERR_NONE;
    {
        std::lock_guard<std::mutex> lock(mtx_radio);
        switch (radio_type) {
            case RadioType::SX1261:
            case RadioType::SX1262:
            case RadioType::SX1268:
                state = ((SX126x*)radio)->setCodingRate(cr);
                break;
            case RadioType::SX1276:
                state = ((SX1276*)radio)->setCodingRate(cr);
                break;
            default:
                state = RADIOLIB_ERR_UNKNOWN;
                break;
        }
        return (state == RADIOLIB_ERR_NONE);
    }
}
bool MeshcoreCompact::setRadioPower(int8_t power) {
    if (radio == nullptr) {
        return false;
    }
    int state = RADIOLIB_ERR_NONE;
    {
        std::lock_guard<std::mutex> lock(mtx_radio);
        state = radio->setOutputPower(power);
    }
    return (state == RADIOLIB_ERR_NONE);
}

bool MeshcoreCompact::RadioInit(RadioType radio_type, Radio_PINS& radio_pins, LoraConfig& lora_config) {
    this->radio_type = radio_type;
    ESP_LOGI(TAG, "RadioInit");
    hal = new EspHal(radio_pins.sck, radio_pins.miso, radio_pins.mosi, radio_pins.cs);
    int state = RADIOLIB_ERR_NONE;
    switch (radio_type) {
        case RadioType::SX1262:
            ESP_LOGI(TAG, "Using SX1262 radio");
            radio = new SX1262(new Module(hal, radio_pins.cs, radio_pins.irq, radio_pins.rst, radio_pins.gpio));
            state = ((SX1262*)radio)->begin(lora_config.frequency, lora_config.bandwidth, lora_config.spreading_factor, lora_config.coding_rate, lora_config.sync_word, lora_config.output_power, lora_config.preamble_length, lora_config.tcxo_voltage, lora_config.use_regulator_ldo);
            break;
        case RadioType::SX1261:
            ESP_LOGI(TAG, "Using SX1261 radio");
            radio = new SX1261(new Module(hal, radio_pins.cs, radio_pins.irq, radio_pins.rst, radio_pins.gpio));
            state = ((SX1261*)radio)->begin(lora_config.frequency, lora_config.bandwidth, lora_config.spreading_factor, lora_config.coding_rate, lora_config.sync_word, lora_config.output_power, lora_config.preamble_length, lora_config.tcxo_voltage, lora_config.use_regulator_ldo);
            break;
        case RadioType::SX1268:
            ESP_LOGI(TAG, "Using SX1268 radio");
            radio = new SX1268(new Module(hal, radio_pins.cs, radio_pins.irq, radio_pins.rst, radio_pins.gpio));
            state = ((SX1268*)radio)->begin(lora_config.frequency, lora_config.bandwidth, lora_config.spreading_factor, lora_config.coding_rate, lora_config.sync_word, lora_config.output_power, lora_config.preamble_length, lora_config.tcxo_voltage, lora_config.use_regulator_ldo);
            break;
        case RadioType::SX1276:
            ESP_LOGI(TAG, "Using SX1276 radio");
            radio = new SX1276(new Module(hal, radio_pins.cs, radio_pins.irq, radio_pins.rst, radio_pins.gpio));
            state = ((SX1276*)radio)->begin(lora_config.frequency, lora_config.bandwidth, lora_config.spreading_factor, lora_config.coding_rate, lora_config.sync_word, lora_config.output_power, lora_config.preamble_length, 5);
            break;
        default:
            ESP_LOGE(TAG, "Unsupported radio type, let's try: SX1262");
            radio = new SX1262(new Module(hal, radio_pins.cs, radio_pins.irq, radio_pins.rst, radio_pins.gpio));
            state = ((SX1262*)radio)->begin(lora_config.frequency, lora_config.bandwidth, lora_config.spreading_factor, lora_config.coding_rate, lora_config.sync_word, lora_config.output_power, lora_config.preamble_length, lora_config.tcxo_voltage, lora_config.use_regulator_ldo);
            return false;
    }

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "failed, code %d\n", state);
        delete hal;
        delete radio;
        hal = nullptr;
        radio = nullptr;
        return false;
    }

    // todo do it less ugly.
    switch (radio_type) {
        case RadioType::SX1261:
            state |= ((SX1261*)radio)->setCurrentLimit(130.0);
            state |= ((SX1261*)radio)->explicitHeader();
            state |= ((SX1261*)radio)->setCRC(RADIOLIB_SX126X_LORA_CRC_ON);
            state |= ((SX1261*)radio)->setDio2AsRfSwitch(false);
            ((SX1261*)radio)->setDio1Action(onPacketReceived);
            state |= ((SX1261*)radio)->setRxBoostedGainMode(true);
            break;
        case RadioType::SX1268:
            state |= ((SX1268*)radio)->setCurrentLimit(130.0);
            state |= ((SX1268*)radio)->explicitHeader();
            state |= ((SX1268*)radio)->setCRC(RADIOLIB_SX126X_LORA_CRC_ON);
            state |= ((SX1268*)radio)->setDio2AsRfSwitch(false);
            ((SX1268*)radio)->setDio1Action(onPacketReceived);
            state |= ((SX1268*)radio)->setRxBoostedGainMode(true);
            break;
        case RadioType::SX1276:
            // todo check
            state |= ((SX1276*)radio)->setCurrentLimit(130.0);
            state |= ((SX1276*)radio)->explicitHeader();
            state |= ((SX1276*)radio)->setCRC(RADIOLIB_SX126X_LORA_CRC_ON);
            // state |= ((SX1276*)radio)->setDio2AsRfSwitch(false);
            ((SX1276*)radio)->setDio1Action(onPacketReceived, 1);
            // state |= ((SX1276*)radio)->setRxBoostedGainMode(true);
            break;
        default:
        case RadioType::SX1262:
            state |= ((SX1262*)radio)->setCurrentLimit(130.0);
            state |= ((SX1262*)radio)->explicitHeader();
            state |= ((SX1262*)radio)->setCRC(RADIOLIB_SX126X_LORA_CRC_ON);
            state |= ((SX1262*)radio)->setDio2AsRfSwitch(false);
            ((SX1262*)radio)->setDio1Action(onPacketReceived);
            state |= ((SX1262*)radio)->setRxBoostedGainMode(true);
            break;
    };
    if (state != 0) {
        ESP_LOGE(TAG, "Radio init failed, code %d\n", state);
        return false;
    }

    RadioListen();    // Start listening for packets
    RadioSendInit();  // Start the send task
    return true;
}

void MeshcoreCompact::task_send(void* pvParameters) {
    MeshcoreCompact* mshcomp = static_cast<MeshcoreCompact*>(pvParameters);
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    while (mshcomp->need_run) {
        /* MCT_OutQueueEntry entry = mshcomp->out_queue.pop();

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
                 if (mshcomp->ok_to_mqtt) entry.data.bitfield |= 1 << BITFIELD_OK_TO_MQTT_SHIFT;  // Set the MQTT upload bit
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
                 MCT_NodeInfo* dstnode = mshcomp->nodeinfo_db.get(entry.header.dstnode);
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
                 mshcomp->radio->startReceive();
             }
         }
             */
        // Restart receiving after sending
        vTaskDelay(350 / portTICK_PERIOD_MS);  // Wait before next send attempt
    }  // end while
    // never reach here
    vTaskDelete(NULL);
}

void MeshcoreCompact::task_listen(void* pvParameters) {
    MeshcoreCompact* mshcomp = static_cast<MeshcoreCompact*>(pvParameters);
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

bool MeshcoreCompact::RadioListen() {
    xTaskCreate(&task_listen, "RadioListen", 1024 * 4, this, 5, NULL);
    return true;
}

bool MeshcoreCompact::RadioSendInit() {
    // Start the send task
    xTaskCreate(&task_send, "RadioSend", 1024 * 4, this, 5, NULL);
    return true;
}

void MeshcoreCompact::intOnNodeInfo(MCC_Nodeinfo& nodeinfo) {
    nodeinfo_db.addOrUpdate(nodeinfo);
    if (onNodeInfo) {
        onNodeInfo(nodeinfo);
    }
    ESP_LOGI(TAG, "timestamp=%lu, flags=0x%02x", nodeinfo.timestamp, nodeinfo.flags);
    if (nodeinfo.flags & (uint8_t)MCC_NODEINFO_FLAGS::HAS_LOCATION) {
        ESP_LOGI(TAG, ", latitude_i=%lu, longitude_i=%lu", nodeinfo.latitude_i, nodeinfo.longitude_i);
    }
    if (nodeinfo.flags & (uint8_t)MCC_NODEINFO_FLAGS::HAS_NAME) {
        ESP_LOGI(TAG, ", name=%s", nodeinfo.name.c_str());
    }
}

int16_t MeshcoreCompact::ProcessPacket(uint8_t* data, int len, MeshcoreCompact* mshcomp) {
    MCC_Header header;
    size_t pos = header.parse(data, len);
    if (pos == 0) {
        ESP_LOGE(TAG, "Failed to parse MCC header");
        return -1;
    }
    ESP_LOGI(TAG, "Received packet: route_type=%d, payload_type=%d, addr_format=%d, transport_codes=0x%08" PRIx32 ", path_length=%zu", (uint8_t)header.get_route_type(), (uint8_t)header.get_payload_type(), (uint8_t)header.get_addr_format(), header.transport_codes, header.path.size());
    ESP_LOGI(TAG, "Path: ");
    for (const auto& hop : header.path) {
        ESP_LOGI(TAG, "  Hop: %ud", hop);
    }
    MCC_PAYLOAD_TYPE plt = header.get_payload_type();

    if (plt == MCC_PAYLOAD_TYPE::PAYLOAD_TYPE_ADVERT) {
        MCC_Nodeinfo nodeinfo;
        if (nodeinfo.parse(data, pos, len) > 0) {
            intOnNodeInfo(nodeinfo);
        }
        return 1;
    }

    if (plt == MCC_PAYLOAD_TYPE::PAYLOAD_TYPE_ACK) {
        uint32_t crc = 0;
        if (len >= pos + 4) {
            crc = *((uint32_t*)&data[pos]);
            ESP_LOGI(TAG, "ACK for CRC=0x%08" PRIx32, crc);
            return 1;
        }
    }

    if (plt == MCC_PAYLOAD_TYPE::PAYLOAD_TYPE_PATH || plt == MCC_PAYLOAD_TYPE::PAYLOAD_TYPE_REQ || plt == MCC_PAYLOAD_TYPE::PAYLOAD_TYPE_RESPONSE || plt == MCC_PAYLOAD_TYPE::PAYLOAD_TYPE_TXT_MSG) {
        uint8_t dst_hash = *((uint8_t*)&data[pos++]);
        uint8_t src_hash = *((uint8_t*)&data[pos++]);
        uint8_t* macanddata = &data[pos];
        uint8_t datadec[MAX_PACKET_PAYLOAD];
        // todo foreach peer list with that dst hash, and check for secret data if i can decrypt with it.
        uint8_t secret[PUB_KEY_SIZE] = {0};  // 32 bytes
        int lenn = MACThenDecrypt(secret, datadec, macanddata, len - pos);
        if (lenn > 0) {
            ESP_LOGI(TAG, "Decrypted payload length: %d", lenn);
        } else {
            ESP_LOGE(TAG, "Failed to decrypt payload");
            return 0;
        }
        pos = 0;  // inner pos
        if (plt == MCC_PAYLOAD_TYPE::PAYLOAD_TYPE_PATH) {
            ESP_LOGI(TAG, "PATH packet:NIY");
            return 0;
        }
        if (plt == MCC_PAYLOAD_TYPE::PAYLOAD_TYPE_REQ) {
            // timestamp 4 byte
            uint32_t timestamp = *((uint32_t*)&datadec[pos]);
            pos += 4;
            uint8_t request_type = datadec[pos++];
            ESP_LOGI(TAG, "REQ packet:NIY");
            return 0;
        }
        if (plt == MCC_PAYLOAD_TYPE::PAYLOAD_TYPE_RESPONSE) {
            // timestamp 4 byte
            uint32_t tag = *((uint32_t*)&datadec[pos]);
            pos += 4;
            ESP_LOGI(TAG, "RESP packet:NIY");
            return 0;
        }
        if (plt == MCC_PAYLOAD_TYPE::PAYLOAD_TYPE_TXT_MSG) {
            // timestamp 4 byte
            uint32_t timestamp = *((uint32_t*)&datadec[pos]);
            pos += 4;
            uint8_t msg_flags = datadec[pos++];
            uint8_t msg_attempts = msg_flags & 0x03;
            msg_flags = msg_flags >> 2;
            if (msg_flags == 2) {
                // signed_plaintext. remove the first 4 bytes: first four bytes is sender pubkey prefix, followed by plain text message
                pos += 4;
            }
            uint16_t msg_len = len - pos;
            std::string msg = std::string((const char*)&datadec[pos], msg_len);
            ESP_LOGI(TAG, "TXT_MSG packet: timestamp=%lu, flags=0x%02x, msg_len=%u, msg=%s", timestamp, msg_flags, msg_len, msg.c_str());
            return 1;
            /*
            Received packet of length 38: 09 00 48 AC A0 13 C8 09 C2 36 BF F6 CC 78 B2 35 18 37 75 7D FC 9B 61 F2 0E 41 15 20 4B 52 C0 B2 55 DF 8A 8B E7 65
I (86779) MeshcoreCompact: Received packet: route_type=1, payload_type=2, addr_format=0, transport_codes=0x00000000, path_length=0
I (86799) MeshcoreCompact: Path:
I (86799) MeshcoreCompact: TXT_MSG packet: timestamp=918686152, flags=0xbf, msg_len=27, msg=��x�57u}��a�A KR��Uߊ��e
Received packet of length 38: 09 00 48 AC 0D 5B 7D C7 81 BF CF 6E 4A 32 00 B8 6A 3E DE E9 F5 B2 61 F2 0E 41 15 20 4B 52 C0 B2 55 DF 8A 8B E7 65
I (96189) MeshcoreCompact: Received packet: route_type=1, payload_type=2, addr_format=0, transport_codes=0x00000000, path_length=0
I (96209) MeshcoreCompact: Path:
I (96209) MeshcoreCompact: TXT_MSG packet: timestamp=3212953469, flags=0xcf, msg_len=27, msg=nJ2
Received packet of length 38: 09 00 48 AC EC D0 0E 15 8C 29 CC 1F AE C7 C8 58 83 A3 CF 5E 39 0F 61 F2 0E 41 15 20 4B 52 C0 B2 55 DF 8A 8B E7 65
I (105999) MeshcoreCompact: Received packet: route_type=1, payload_type=2, addr_format=0, transport_codes=0x00000000, path_length=0
I (106019) MeshcoreCompact: Path:
I (106019) MeshcoreCompact: TXT_MSG packet: timestamp=697046286, flags=0xcc, msg_len=27, msg=���X���^9a�A KR��Uߊ��e
*/
        }
    }

    if (plt == MCC_PAYLOAD_TYPE::PAYLOAD_TYPE_ANON_REQ) {
        ESP_LOGI(TAG, "PAYLOAD_TYPE_ANON_REQ NIY");
        return 0;
    }
    if (plt == MCC_PAYLOAD_TYPE::PAYLOAD_TYPE_GRP_TXT) {
        uint8_t chan_hash = data[pos++];
        uint16_t mac = *((uint16_t*)&data[pos]);
        pos += 2;
        // text: encrypted
        ESP_LOGI(TAG, "PAYLOAD_TYPE_GRP_TXT NIY");
        return 0;
    }
    return 0;
}

void MeshcoreCompact::sha256(uint8_t* hash, size_t hash_len, const uint8_t* msg, int msg_len) {
    mbedtls_sha256_context sha256;
    mbedtls_sha256_init(&sha256);
    mbedtls_sha256_starts(&sha256, 0);
    int ret = mbedtls_sha256_update(&sha256, msg, msg_len);
    ret = mbedtls_sha256_finish(&sha256, hash);
    (void)ret;
    mbedtls_sha256_free(&sha256);
};

void MeshcoreCompact::sha256(uint8_t* hash, size_t hash_len, const uint8_t* frag1, int frag1_len, const uint8_t* frag2, int frag2_len) {
    mbedtls_sha256_context sha256;
    mbedtls_sha256_init(&sha256);
    mbedtls_sha256_starts(&sha256, 0);
    int ret = mbedtls_sha256_update(&sha256, frag1, frag1_len);
    ret = mbedtls_sha256_update(&sha256, frag2, frag2_len);
    ret = mbedtls_sha256_finish(&sha256, hash);
    (void)ret;
    mbedtls_sha256_free(&sha256);
};

int MeshcoreCompact::decrypt(const uint8_t* shared_secret, uint8_t* dest, const uint8_t* src, int src_len) {
    if (src_len % 16 != 0) {
        // Or handle this error appropriately
        return 0;
    }
    mbedtls_aes_context aes_ctx;
    uint8_t* dp = dest;
    const uint8_t* sp = src;
    // Initialize the AES context
    mbedtls_aes_init(&aes_ctx);
    // Set the decryption key. Note the use of `_setkey_dec`
    // IMPORTANT: CIPHER_KEY_SIZE must be in BITS (e.g., 128, 192, 256)
    mbedtls_aes_setkey_dec(&aes_ctx, shared_secret, CIPHER_KEY_SIZE * 8);

    // Process all 16-byte blocks
    for (int i = 0; i < src_len; i += 16) {
        mbedtls_aes_crypt_ecb(
            &aes_ctx,             // AES context
            MBEDTLS_AES_DECRYPT,  // Decrypt mode
            sp,                   // Source ciphertext block
            dp                    // Destination plaintext block
        );
        dp += 16;
        sp += 16;
    }
    // Clean up the context
    mbedtls_aes_free(&aes_ctx);
    // Return the total number of bytes processed
    return sp - src;
};
int MeshcoreCompact::encrypt(const uint8_t* shared_secret, uint8_t* dest, const uint8_t* src, int src_len) {
    mbedtls_aes_context aes_ctx;
    uint8_t* dp = dest;
    const uint8_t* sp = src;
    int remaining_len = src_len;
    // Initialize the AES context
    mbedtls_aes_init(&aes_ctx);
    // Set the encryption key.
    // IMPORTANT: CIPHER_KEY_SIZE must be in BITS (e.g., 128, 192, 256)
    mbedtls_aes_setkey_enc(&aes_ctx, shared_secret, CIPHER_KEY_SIZE * 8);
    // Process all full 16-byte blocks
    while (remaining_len >= 16) {
        mbedtls_aes_crypt_ecb(
            &aes_ctx,             // AES context
            MBEDTLS_AES_ENCRYPT,  // Encrypt mode
            sp,                   // Source block
            dp                    // Destination block
        );
        dp += 16;
        sp += 16;
        remaining_len -= 16;
    }

    // Handle the final partial block with zero padding
    if (remaining_len > 0) {
        uint8_t tmp_block[16];
        // Pad the temporary block with zeros
        memset(tmp_block, 0, 16);
        // Copy the remaining plaintext into the block
        memcpy(tmp_block, sp, remaining_len);
        // Encrypt the padded block
        mbedtls_aes_crypt_ecb(
            &aes_ctx,
            MBEDTLS_AES_ENCRYPT,
            tmp_block,
            dp);
        dp += 16;
    }
    // Clean up the context
    mbedtls_aes_free(&aes_ctx);
    // Return the total number of bytes written to the destination
    return dp - dest;
};
int MeshcoreCompact::encryptThenMAC(const uint8_t* shared_secret, uint8_t* dest, const uint8_t* src, int src_len) {
    int enc_len = encrypt(shared_secret, dest + CIPHER_MAC_SIZE, src, src_len);
    const mbedtls_md_info_t* md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    // This single function performs the reset, update, and finalize steps.
    int ret = mbedtls_md_hmac(
        md_info,                 // Use SHA256
        shared_secret,           // The key for the HMAC
        PUB_KEY_SIZE,            // The length of the key
        dest + CIPHER_MAC_SIZE,  // The message to authenticate
        enc_len,                 // The length of the message
        dest                     // The destination for the 32-byte MAC output
    );
    return 0;
};
int MeshcoreCompact::MACThenDecrypt(const uint8_t* shared_secret, uint8_t* dest, const uint8_t* src, int src_len) {
    if (src_len <= CIPHER_MAC_SIZE) {
        return 0;  // Invalid source length
    }
    uint8_t calculated_mac[CIPHER_MAC_SIZE];
    const uint8_t* received_mac = src;
    const uint8_t* ciphertext = src + CIPHER_MAC_SIZE;
    const int ciphertext_len = src_len - CIPHER_MAC_SIZE;
    // 2. Calculate the HMAC of the ciphertext portion.
    const mbedtls_md_info_t* md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (md_info == NULL) {
        return 0;  // Internal error
    }
    int ret = mbedtls_md_hmac(
        md_info,         // Use SHA256
        shared_secret,   // The HMAC key
        PUB_KEY_SIZE,    // Key length
        ciphertext,      // The data to authenticate
        ciphertext_len,  // Length of the data
        calculated_mac   // Output buffer for the calculated MAC
    );

    if (ret != 0) {
        return 0;  // HMAC calculation failed
    }
    // 3. 🛡️ Securely compare the received MAC with the calculated MAC.
    if (secure_memcmp(received_mac, calculated_mac, CIPHER_MAC_SIZE) == 0) {
        // 4. If MAC is valid, decrypt the ciphertext.
        return decrypt(shared_secret, dest, ciphertext, ciphertext_len);
    }

    // If MACs do not match, return 0 to indicate authentication failure.
    return 0;
};

int MeshcoreCompact::secure_memcmp(const void* a, const void* b, size_t size) {
    const unsigned char* a_ptr = (const unsigned char*)a;
    const unsigned char* b_ptr = (const unsigned char*)b;
    unsigned int result = 0;

    // This loop always runs for the full 'size' iterations.
    // The bitwise operations prevent the compiler from optimizing it
    // into a branch that could leak timing information.
    for (size_t i = 0; i < size; i++) {
        result |= a_ptr[i] ^ b_ptr[i];
    }

    return result;
}
