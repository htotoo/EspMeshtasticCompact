#ifndef MESHTASTIC_COMPACT_H
#define MESHTASTIC_COMPACT_H

#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <string.h>
#include "pb.h"
#include "RadioLib.h"
#include "EspHal.h"
#include "esp_random.h"
#include "mbedtls/aes.h"
#include <string>
#include "meshtastic/mesh.pb.h"
#include "MeshtasticCompactStructs.hpp"
#include <mutex>
#include <condition_variable>
#include <queue>
#include <deque>
#include "MeshtasticCompactNodeInfoDB.hpp"
#include "MeshtasticCompactRouter.hpp"
#include "MeshtasticCompactOutQueue.hpp"
#include "MeshtasticCompactFileIO.hpp"

// https://github.com/meshtastic/firmware/blob/81828c6244daede254cf759a0f2bd939b2e7dd65/variants/heltec_wsl_v3/variant.h

class MeshtasticCompact {
   public:
    MeshtasticCompact();
    ~MeshtasticCompact();
    bool RadioInit(RadioType radio_type, Radio_PINS& radio_pins, LoraConfig& lora_config);  // Initializes the radio with the given configuration and pins

    // callbacks
    using OnMessageCallback = void (*)(MCT_Header& header, MCT_TextMessage& message);
    using OnPositionMessageCallback = void (*)(MCT_Header& header, MCT_Position& position, bool needReply);
    using OnNodeInfoCallback = void (*)(MCT_Header& header, MCT_NodeInfo& nodeinfo, bool needReply, bool newNode);
    using OnWaypointMessageCallback = void (*)(MCT_Header& header, MCT_Waypoint& waypoint);
    using OnTelemetryDeviceCallback = void (*)(MCT_Header& header, MCT_Telemetry_Device& telemetry);
    using OnTelemetryEnvironmentCallback = void (*)(MCT_Header& header, MCT_Telemetry_Environment& telemetry);
    using OnTracerouteCallback = void (*)(MCT_Header& header, MCT_RouteDiscovery& route, bool for_me, bool is_reply, bool need_reply);
    using OnRaw = void (*)(const uint8_t* data, size_t len);
    using OnNativePositionMessageCallback = void (*)(MCT_Header& header, meshtastic_Position& position, bool needReply);
    using OnNativeNodeInfoCallback = void (*)(MCT_Header& header, meshtastic_User& nodeinfo, bool needReply, bool newNode);
    using OnNativeWaypointMessageCallback = void (*)(MCT_Header& header, meshtastic_Waypoint& waypoint);
    using OnNativeTelemetryDeviceCallback = void (*)(MCT_Header& header, meshtastic_DeviceMetrics& telemetry);
    using OnNativeTelemetryEnvironmentCallback = void (*)(MCT_Header& header, meshtastic_EnvironmentMetrics& telemetry);

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

    /**
     * @brief Get the Last Signal Strengh Data
     *
     * @param rssi_out RSSI value
     * @param snr_out SNR value
     */
    void getLastSignalData(float& rssi_out, float& snr_out) {
        rssi_out = rssi;
        snr_out = snr;
    }

    /**
     * @brief Set the Send Enabled
     *
     * @param enabled If set to false, no packets will be sent, but we still receive them.
     */
    void setSendEnabled(bool enabled) {
        is_send_enabled = enabled;
    }
    void setAutoFullNode(bool enabled) { is_auto_full_node = enabled; }  // if true, sends ack, traceroute reply, and position reply automatically

    /**
     * @brief Set the Send Hop Limit object
     *
     * @param limit
     * @return true Valid hop limit set
     * @return false Invalid hop limit, must be between 1 and 7
     */
    bool setSendHopLimit(uint8_t limit) {
        if (limit > 0 && limit <= 7) {
            send_hop_limit = limit;
            return true;
        }
        return false;
    }

    /**
     * @brief Set the Stealth Mode
     *
     * @param stealth If set to true, we don't respond to traceroute even in auto full node mode! harder to find us. We even don't send ack.
     */
    void setStealthMode(bool stealth) {
        is_in_stealth_mode = stealth;
    }

    /**
     * @brief Set the My Names object
     *
     * @param short_name
     * @param long_name
     */
    void setMyNames(const char* short_name, const char* long_name);
    MCT_NodeInfo* getMyNodeInfo() {
        return &my_nodeinfo;
    }

    void set_ok_to_mqtt(bool ok) { ok_to_mqtt = ok; }  // if true, sets the flag in the header

    // packet senders
    void SendNodeInfo(MCT_NodeInfo& nodeinfo, uint32_t dstnode = 0xffffffff, bool exchange = false);
    void SendMyNodeInfo(uint32_t dstnode = 0xffffffff, bool exchange = false) {
        SendNodeInfo(my_nodeinfo, dstnode, exchange);
    }
    void SendTextMessage(const std::string& text, uint32_t dstnode = 0xffffffff, uint8_t chan = 0, MCT_MESSAGE_TYPE type = MCT_MESSAGE_TYPE_TEXT, uint32_t sender_node_id = 0);
    void SendPositionMessage(MCT_Position& position, uint32_t dstnode = 0xffffffff, uint8_t chan = 8, uint32_t sender_node_id = 0);
    void SendMyPosition(uint32_t dstnode = 0xffffffff, uint8_t chan = 8) {
        SendPositionMessage(my_position, dstnode, chan);
    }
    void SendRequestPositionInfo(uint32_t dest_node_id, uint8_t chan = 8, uint32_t sender_node_id = 0);
    void SendWaypointMessage(MCT_Waypoint& waypoint, uint32_t dstnode = 0xffffffff, uint8_t chan = 8, uint32_t sender_node_id = 0);
    void SendTelemetryDevice(MCT_Telemetry_Device& telemetry, uint32_t dstnode = 0xffffffff, uint8_t chan = 8, uint32_t sender_node_id = 0);
    void SendTelemetryEnvironment(MCT_Telemetry_Environment& telemetry, uint32_t dstnode = 0xffffffff, uint8_t chan = 8, uint32_t sender_node_id = 0);
    void SendTracerouteReply(MCT_Header& header, MCT_RouteDiscovery& route_discovery);
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
    MCT_Position my_position;        // My position, used for auto replies (when enabled) on position requests. Or when you call SendMyPosition()
   private:
    RadioType radio_type;
    bool RadioListen();    // inits the listening thread for the radio
    bool RadioSendInit();  // inits the sending thread for the radio. consumes the out_queue
    // handlers
    void intOnMessage(MCT_Header& header, MCT_TextMessage& message);                                // Called when got any text based messages
    void intOnPositionMessage(MCT_Header& header, meshtastic_Position& position, bool want_reply);  // Called on position messages
    void intOnNodeInfo(MCT_Header& header, meshtastic_User& user_msg, bool want_reply);             // Called on node info messages
    void intOnWaypointMessage(MCT_Header& header, meshtastic_Waypoint& waypoint_msg);               // Called on waypoint messages
    void intOnTelemetryDevice(MCT_Header& header, _meshtastic_Telemetry& telemetry);                // Called on telemetry device messages
    void intOnTelemetryEnvironment(MCT_Header& header, _meshtastic_Telemetry& telemetry_msg);       // Called on telemetry environment messages
    void intOnTraceroute(MCT_Header& header, meshtastic_RouteDiscovery& route_discovery_msg);       // Called on traceroute messages

    // mesh network minimum functionality

    void send_ack(MCT_Header& header);  // sends an ack packet to the source node based on the header

    // decoding
    int16_t ProcessPacket(uint8_t* data, int len, MeshtasticCompact* mshcomp);  // Process the packet, decode it, and call the appropriate handler

    int16_t try_decode_root_packet(const uint8_t* srcbuf, size_t srcbufsize, const pb_msgdesc_t* fields, void* dest_struct, size_t dest_struct_size, MCT_Header& header);                // the simple packet decoder for any type of encrypted messages.
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

    MCT_NodeInfo my_nodeinfo;  // My node info. Used in many places. Set it carefully.

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

/**
 * @brief Simple static helpers for building message objects easily.
 *
 */
class MeshtasticCompactHelpers {
   public:
    static void NodeInfoBuilder(MCT_NodeInfo* nodeinfo, uint32_t node_id, std::string& short_name, std::string& long_name, uint8_t hw_model);
    static void PositionBuilder(MCT_Position& position, float latitude, float longitude, int32_t altitude = 0, uint32_t speed = 0, uint32_t sats_in_view = 0);
    static void TelemetryDeviceBuilder(MCT_Telemetry_Device& telemetry, uint32_t uptime_seconds = 0, float voltage = 0.0f, float battery_level = -1.0f, float channel_utilization = -1.0f);
    static void TelemetryEnvironmentBuilder(MCT_Telemetry_Environment& telemetry, float temperature = -10000.0f, float humidity = -1.0f, float pressure = -1.0f, float lux = -1.0f);
    static void WaypointBuilder(MCT_Waypoint& waypoint, uint32_t id, float latitude, float longitude, std::string name, std::string description, uint32_t expire = 1, uint32_t icon = 0);
};

#endif  // MESHTASTIC_COMPACT_H