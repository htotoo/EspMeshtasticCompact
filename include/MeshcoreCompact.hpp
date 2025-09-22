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

    /*    // callbacks
        using OnMessageCallback = void (*)(MC_Header& header, MC_TextMessage& message);
        using OnPositionMessageCallback = void (*)(MC_Header& header, MC_Position& position, bool needReply);
        using OnNodeInfoCallback = void (*)(MC_Header& header, MC_NodeInfo& nodeinfo, bool needReply, bool newNode);
        using OnWaypointMessageCallback = void (*)(MC_Header& header, MC_Waypoint& waypoint);
        using OnTelemetryDeviceCallback = void (*)(MC_Header& header, MC_Telemetry_Device& telemetry);
        using OnTelemetryEnvironmentCallback = void (*)(MC_Header& header, MC_Telemetry_Environment& telemetry);
        using OnTracerouteCallback = void (*)(MC_Header& header, MC_RouteDiscovery& route, bool for_me, bool is_reply, bool need_reply);
        using OnRaw = void (*)(const uint8_t* data, size_t len);
        using OnNativePositionMessageCallback = void (*)(MC_Header& header, meshtastic_Position& position, bool needReply);
        using OnNativeNodeInfoCallback = void (*)(MC_Header& header, meshtastic_User& nodeinfo, bool needReply, bool newNode);
        using OnNativeWaypointMessageCallback = void (*)(MC_Header& header, meshtastic_Waypoint& waypoint);
        using OnNativeTelemetryDeviceCallback = void (*)(MC_Header& header, meshtastic_DeviceMetrics& telemetry);
        using OnNativeTelemetryEnvironmentCallback = void (*)(MC_Header& header, meshtastic_EnvironmentMetrics& telemetry);

        void setOnWaypointMessage(OnWaypointMessageCallback cb) { onWaypointMessage = cb; }
        void setOnNodeInfoMessage(OnNodeInfoCallback cb) { onNodeInfo = cb; }
        void setOnPositionMessage(OnPositionMessageCallback cb) { onPositionMessage = cb; }
        void setOnMessage(OnMessageCallback cb) { onMessage = cb; }
        void setOnTelemetryDevice(OnTelemetryDeviceCallback cb) { onTelemetryDevice = cb; }
        void setOnTelemetryEnvironment(OnTelemetryEnvironmentCallback cb) { onTelemetryEnvironment = cb; }
        void setOnTraceroute(OnTracerouteCallback cb) { onTraceroute = cb; }
        void setOnRaw(OnRaw cb) { onRaw = cb; }
        void setOnNativePositionMessage(OnNativePositionMessageCallback cb) { onNativePositionMessage = cb; }
        void setOnNativeNodeInfo(OnNativeNodeInfoCallback cb) { onNativeNodeInfo = cb; }
        void setOnNativeWaypointMessage(OnNativeWaypointMessageCallback cb) { onNativeWaypointMessage = cb; }
        void setOnNativeTelemetryDevice(OnNativeTelemetryDeviceCallback cb) { onNativeTelemetryDevice = cb; }
        void setOnNativeTelemetryEnvironment(OnNativeTelemetryEnvironmentCallback cb) { onNativeTelemetryEnvironment = cb; }


    void getLastSignalData(float& rssi_out, float& snr_out) {
        rssi_out = rssi;
        snr_out = snr;
    }


    void setSendEnabled(bool enabled) {
        is_send_enabled = enabled;
    }
    void setAutoFullNode(bool enabled) { is_auto_full_node = enabled; }  // if true, sends ack, traceroute reply, and position reply automatically


    bool setSendHopLimit(uint8_t limit) {
        if (limit > 0 && limit <= 7) {
            send_hop_limit = limit;
            return true;
        }
        return false;
    }


    void setStealthMode(bool stealth) {
        is_in_stealth_mode = stealth;
    }


    void setMyNames(const char* short_name, const char* long_name);
    MC_NodeInfo* getMyNodeInfo() {
        return &my_nodeinfo;
    }

    void set_ok_to_mqtt(bool ok) { ok_to_mqtt = ok; }  // if true, sets the flag in the header

    // packet senders
    void SendNodeInfo(MC_NodeInfo& nodeinfo, uint32_t dstnode = 0xffffffff, bool exchange = false);
    void SendMyNodeInfo(uint32_t dstnode = 0xffffffff, bool exchange = false) {
        SendNodeInfo(my_nodeinfo, dstnode, exchange);
    }
    void SendTextMessage(const std::string& text, uint32_t dstnode = 0xffffffff, uint8_t chan = 0, MC_MESSAGE_TYPE type = MC_MESSAGE_TYPE_TEXT, uint32_t sender_node_id = 0);
    void SendPositionMessage(MC_Position& position, uint32_t dstnode = 0xffffffff, uint8_t chan = 8, uint32_t sender_node_id = 0);
    void SendMyPosition(uint32_t dstnode = 0xffffffff, uint8_t chan = 8) {
        SendPositionMessage(my_position, dstnode, chan);
    }
    void SendRequestPositionInfo(uint32_t dest_node_id, uint8_t chan = 8, uint32_t sender_node_id = 0);
    void SendWaypointMessage(MC_Waypoint& waypoint, uint32_t dstnode = 0xffffffff, uint8_t chan = 8, uint32_t sender_node_id = 0);
    void SendTelemetryDevice(MC_Telemetry_Device& telemetry, uint32_t dstnode = 0xffffffff, uint8_t chan = 8, uint32_t sender_node_id = 0);
    void SendTelemetryEnvironment(MC_Telemetry_Environment& telemetry, uint32_t dstnode = 0xffffffff, uint8_t chan = 8, uint32_t sender_node_id = 0);
    void SendTracerouteReply(MC_Header& header, MC_RouteDiscovery& route_discovery);
    void SendTraceroute(uint32_t dest_node_id, uint8_t chan = 8, uint32_t sender_node_id = 0);

    // Radio settings on the fly
    bool setRadioFrequency(float freq);
    bool setRadioSpreadingFactor(uint8_t sf);
    bool setRadioBandwidth(uint32_t bw);
    bool setRadioCodingRate(uint8_t cr);
    bool setRadioPower(int8_t power);

    void saveNodeDb() {
        MeshtasticCompactFileIO::saveNodeDb(nodeinfo_db);
    }
    void loadNodeDb() {
        MeshtasticCompactFileIO::loadNodeDb(nodeinfo_db);
    }

    NodeInfoDB nodeinfo_db;          // NodeInfo database.
    MeshtasticCompactRouter router;  // Router for message deduplication. Set MyId if you changed that. Also you can disable exclude self option
    MC_Position my_position;         // My position, used for auto replies (when enabled) on position requests. Or when you call SendMyPosition()

   */
   private:
    RadioType radio_type;
    bool RadioListen();    // inits the listening thread for the radio
    bool RadioSendInit();  // inits the sending thread for the radio. consumes the out_queue
    // handlers
    void intOnMessage(MC_Header& header, MC_TextMessage& message);                                 // Called when got any text based messages
    void intOnPositionMessage(MC_Header& header, meshtastic_Position& position, bool want_reply);  // Called on position messages
    void intOnNodeInfo(MC_Header& header, meshtastic_User& user_msg, bool want_reply);             // Called on node info messages
    void intOnWaypointMessage(MC_Header& header, meshtastic_Waypoint& waypoint_msg);               // Called on waypoint messages
    void intOnTelemetryDevice(MC_Header& header, _meshtastic_Telemetry& telemetry);                // Called on telemetry device messages
    void intOnTelemetryEnvironment(MC_Header& header, _meshtastic_Telemetry& telemetry_msg);       // Called on telemetry environment messages
    void intOnTraceroute(MC_Header& header, meshtastic_RouteDiscovery& route_discovery_msg);       // Called on traceroute messages

    // mesh network minimum functionality

    void send_ack(MC_Header& header);  // sends an ack packet to the source node based on the header

    // decoding
    int16_t ProcessPacket(uint8_t* data, int len, MeshtasticCompact* mshcomp);  // Process the packet, decode it, and call the appropriate handler

    int16_t try_decode_root_packet(const uint8_t* srcbuf, size_t srcbufsize, const pb_msgdesc_t* fields, void* dest_struct, size_t dest_struct_size, MC_Header& header);                 // the simple packet decoder for any type of encrypted messages.
    bool pb_decode_from_bytes(const uint8_t* srcbuf, size_t srcbufsize, const pb_msgdesc_t* fields, void* dest_struct);                                                                  // decode the protobuf message from bytes
    size_t pb_encode_to_bytes(uint8_t* destbuf, size_t destbufsize, const pb_msgdesc_t* fields, const void* src_struct);                                                                 // encode the protobuf message to bytes
    static void task_listen(void* pvParameters);                                                                                                                                         // Task for listening to the radio and processing incoming packets
    static void task_send(void* pvParameters);                                                                                                                                           // Task for sending packets from the out_queue
    bool aes_decrypt_meshtastic_payload(const uint8_t* key, uint16_t keySize, uint32_t packet_id, uint32_t from_node, const uint8_t* encrypted_in, uint8_t* decrypted_out, size_t len);  // decrypts the meshtastic payload using AES
    uint8_t getLastByteOfNodeNum(uint32_t num);

    float rssi, snr;                  // store last signal data
    bool is_send_enabled = true;      // global sending enabled flag. if false, noone can send anything
    uint8_t send_hop_limit = 7;       // default hop limit for sending packets
    bool is_auto_full_node = true;    // if true, we automatically send ack, traceroute reply, and position reply when needed
    bool is_in_stealth_mode = false;  // if true, we don't respond to traceroute even in auto full node mode! harder to find us. We even don't send ack.

    bool ok_to_mqtt = true;  // set or don't set the flag.

    MC_NodeInfo my_nodeinfo;  // My node info. Used in many places. Set it carefully.

    EspHal* hal;           // = new EspHal(9, 11, 10);
    PhysicalLayer* radio;  // SX1262 radio = new Module(hal, 8, 14, 12, 13);

    const uint8_t default_l1_key[16] =
        {0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
         0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01};  // default aes128 key for L1 encryption
    const uint8_t default_chan_key[32] = {
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // default channel key

    mbedtls_aes_context aes_ctx;
    mutable std::mutex mtx_radio;
    bool need_run = true;  // thread exit flag

    MeshCompactOutQueue out_queue;  // Outgoing queue for packets to be sent.

    // Callback function pointers
    OnMessageCallback onMessage = nullptr;  // Function pointer for onMessage callback
    OnPositionMessageCallback onPositionMessage = nullptr;
    OnNodeInfoCallback onNodeInfo = nullptr;
    OnWaypointMessageCallback onWaypointMessage = nullptr;
    OnTelemetryDeviceCallback onTelemetryDevice = nullptr;
    OnTelemetryEnvironmentCallback onTelemetryEnvironment = nullptr;
    OnTracerouteCallback onTraceroute = nullptr;
    OnRaw onRaw = nullptr;
    OnNativePositionMessageCallback onNativePositionMessage = nullptr;
    OnNativeNodeInfoCallback onNativeNodeInfo = nullptr;
    OnNativeWaypointMessageCallback onNativeWaypointMessage = nullptr;
    OnNativeTelemetryDeviceCallback onNativeTelemetryDevice = nullptr;
    OnNativeTelemetryEnvironmentCallback onNativeTelemetryEnvironment = nullptr;
};
