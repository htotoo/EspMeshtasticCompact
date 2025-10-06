#pragma once

#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <string.h>
#include "RadioStructs.hpp"
#include "RadioLib.h"
#include "EspHal.h"
#include "esp_random.h"
#include "mbedtls/aes.h"
#include "mbedtls/md.h"
#include <string>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <deque>
#include "MeshcoreCompactStructs.hpp"
#include "MeshcoreCompactNodeInfoDB.hpp"
#include "mbedtls/sha256.h"
#include "mbedtls/constant_time.h"

#define MAX_PACKET_PAYLOAD 184
#define PUB_KEY_SIZE 32
#define PRV_KEY_SIZE 64
#define SEED_SIZE 32
#define SIGNATURE_SIZE 64
#define CIPHER_KEY_SIZE 16
#define CIPHER_BLOCK_SIZE 16
#define CIPHER_MAC_SIZE 2

class MeshcoreCompact {
   public:
    MeshcoreCompact();
    ~MeshcoreCompact();
    bool RadioInit(RadioType radio_type, Radio_PINS& radio_pins, LoraConfig& lora_config);  // Initializes the radio with the given configuration and pins

    using OnRaw = void (*)(const uint8_t* data, size_t len);
    using OnNodeInfo = void (*)(const MCC_Nodeinfo& info);

    void setOnRaw(OnRaw cb) { onRaw = cb; }
    void setOnNodeInfo(OnNodeInfo cb) { onNodeInfo = cb; }

    void getLastSignalData(float& rssi_out, float& snr_out) {
        rssi_out = rssi;
        snr_out = snr;
    }

    // Radio settings on the fly
    bool setRadioFrequency(float freq);
    bool setRadioSpreadingFactor(uint8_t sf);
    bool setRadioBandwidth(uint32_t bw);
    bool setRadioCodingRate(uint8_t cr);
    bool setRadioPower(int8_t power);

    NodeInfoCoreDB nodeinfo_db;

   private:
    RadioType radio_type;
    bool RadioListen();    // inits the listening thread for the radio
    bool RadioSendInit();  // inits the sending thread for the radio. consumes the out_queue
                           // handlers

    // internal events
    void intOnNodeInfo(MCC_Nodeinfo& info);  // internal handler for nodeinfo packets

    // decoding
    int16_t ProcessPacket(uint8_t* data, int len, MeshcoreCompact* mshcomp);  // Process the packet, decode it, and call the appropriate handler
    void sha256(uint8_t* hash, size_t hash_len, const uint8_t* msg, int msg_len);
    void sha256(uint8_t* hash, size_t hash_len, const uint8_t* frag1, int frag1_len, const uint8_t* frag2, int frag2_len);
    int decrypt(const uint8_t* shared_secret, uint8_t* dest, const uint8_t* src, int src_len);
    int encrypt(const uint8_t* shared_secret, uint8_t* dest, const uint8_t* src, int src_len);
    int encryptThenMAC(const uint8_t* shared_secret, uint8_t* dest, const uint8_t* src, int src_len);
    int MACThenDecrypt(const uint8_t* shared_secret, uint8_t* dest, const uint8_t* src, int src_len);
    int secure_memcmp(const void* a, const void* b, size_t size);

    static void task_listen(void* pvParameters);  // Task for listening to the radio and processing incoming packets
    static void task_send(void* pvParameters);    // Task for sending packets from the out_queue

    float rssi, snr;  // store last signal data

    EspHal* hal;           // = new EspHal(9, 11, 10);
    PhysicalLayer* radio;  // SX1262 radio = new Module(hal, 8, 14, 12, 13);

    mutable std::mutex mtx_radio;
    bool need_run = true;  // thread exit flag

    OnRaw onRaw = nullptr;
    OnNodeInfo onNodeInfo = nullptr;
};
