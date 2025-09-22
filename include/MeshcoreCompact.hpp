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
#include <string>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <deque>

class MeshcoreCompact {
   public:
    MeshcoreCompact();
    ~MeshcoreCompact();
    bool RadioInit(RadioType radio_type, Radio_PINS& radio_pins, LoraConfig& lora_config);  // Initializes the radio with the given configuration and pins
    using OnRaw = void (*)(const uint8_t* data, size_t len);

    void setOnRaw(OnRaw cb) { onRaw = cb; }

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

   private:
    RadioType radio_type;
    bool RadioListen();    // inits the listening thread for the radio
    bool RadioSendInit();  // inits the sending thread for the radio. consumes the out_queue
    // handlers

    // decoding
    // int16_t ProcessPacket(uint8_t* data, int len, MeshtasticCompact* mshcomp);  // Process the packet, decode it, and call the appropriate handler

    // encode the protobuf message to bytes
    static void task_listen(void* pvParameters);  // Task for listening to the radio and processing incoming packets
    static void task_send(void* pvParameters);    // Task for sending packets from the out_queue

    float rssi, snr;  // store last signal data

    EspHal* hal;           // = new EspHal(9, 11, 10);
    PhysicalLayer* radio;  // SX1262 radio = new Module(hal, 8, 14, 12, 13);

    mutable std::mutex mtx_radio;
    bool need_run = true;  // thread exit flag

    OnRaw onRaw = nullptr;
};
