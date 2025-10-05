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

int16_t MeshcoreCompact::ProcessPacket(uint8_t* data, int len, MeshcoreCompact* mshcomp) {
    MCC_HEADER header;
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
    return 0;
}