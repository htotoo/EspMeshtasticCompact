#ifndef MESHTASTIC_COMPACT_H
#define MESHTASTIC_COMPACT_H

#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <string.h>
#include "pb.h"
#include "RadioLib.h"
#include "EspHal.h"
#include "mbedtls/aes.h"
#include <string>
#include "meshtastic/mesh.pb.h"
#include "MeshasticCompactStructs.hpp"
#include <mutex>
#include <condition_variable>
#include <queue>
#include <deque>
// https://github.com/meshtastic/firmware/blob/81828c6244daede254cf759a0f2bd939b2e7dd65/variants/heltec_wsl_v3/variant.h

/**
 * @brief Handles an in-memory database of node information.
 *
 */
class NodeInfoDB {
   public:
    static constexpr size_t MAX_NODES = 32;

    // Iterator for NodeInfoDB
    class iterator {
       public:
        iterator(MC_NodeInfo* nodeinfos, bool* valid, size_t idx)
            : nodeinfos_(nodeinfos), valid_(valid), idx_(idx) {
            advance_to_valid();
        }
        iterator& operator++() {
            ++idx_;
            advance_to_valid();
            return *this;
        }
        MC_NodeInfo& operator*() {
            assert(idx_ < MAX_NODES && "Attempted to dereference an end() or invalid iterator");
            return nodeinfos_[idx_];
        }
        MC_NodeInfo* operator->() {
            assert(idx_ < MAX_NODES && "Attempted to dereference an end() or invalid iterator");
            return &nodeinfos_[idx_];
        }
        bool operator!=(const iterator& other) const { return idx_ != other.idx_; }
        bool operator==(const iterator& other) const { return idx_ == other.idx_; }

       private:
        void advance_to_valid() {
            while (idx_ < MAX_NODES && !valid_[idx_]) ++idx_;
        }
        MC_NodeInfo* nodeinfos_;
        bool* valid_;
        size_t idx_;
    };

    iterator begin() { return iterator(nodeinfos, valid, 0); }
    iterator end() { return iterator(nodeinfos, valid, MAX_NODES); }

    /**
     * @brief Returns a pointer to the node information at the given index.
     * If the index is out of bounds or not valid, returns nullptr.
     *
     * @param index
     * @return MC_NodeInfo*
     */
    MC_NodeInfo* getByIndex(size_t index) {
        if (index < MAX_NODES && valid[index]) {
            return &nodeinfos[index];
        }
        return nullptr;
    }

    /**
     * @brief Get the Random Node object
     *
     * @return MC_NodeInfo*
     */
    MC_NodeInfo* getRandomNode() {
        uint8_t cnt = 0;
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i]) {
                ++cnt;
            }
        }
        if (cnt == 0) return nullptr;
        size_t target = esp_random() % cnt;
        cnt = 0;
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i]) {
                if (cnt == target) {
                    return &nodeinfos[i];
                }
                cnt++;
            }
        }
        return nullptr;  // should never reach here
    }

    /**
     * @brief Adds new entry or updates existing node information.
     * If the node already exists, it updates the information. If no space, uses LRU policy to overwrite the oldest entry.
     *
     * @param node_id
     * @param info
     */
    void
    addOrUpdate(uint32_t node_id, const MC_NodeInfo& info) {
        // Try to update existing
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i].node_id == node_id) {
                nodeinfos[i] = info;
                return;
            }
        }
        // Add new if space
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (!valid[i]) {
                nodeinfos[i] = info;
                valid[i] = true;
                is_position_valid[i] = false;
                return;
            }
        }
        // No space:  LRU (overwrite the oldest entry)
        size_t oldest_idx = 0;
        uint32_t oldest_time = nodeinfos[0].last_updated;
        for (size_t i = 1; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i].last_updated < oldest_time) {
                oldest_time = nodeinfos[i].last_updated;
                oldest_idx = i;
            }
        }
        nodeinfos[oldest_idx] = info;
        // reset position validity for overwritten node
        is_position_valid[oldest_idx] = false;
    }

    /**
     * @brief Returns a pointer to the node information for the given node_id.
     *
     * @param node_id
     * @return MC_NodeInfo*
     */
    MC_NodeInfo* get(uint32_t node_id) {
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i].node_id == node_id) {
                return &nodeinfos[i];
            }
        }
        return nullptr;
    }

    /**
     * @brief Removes a node entry by node_id. (Along with position if exists)
     *
     * @param node_id
     */
    void remove(uint32_t node_id) {
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i].node_id == node_id) {
                valid[i] = false;
                nodeinfos[i].node_id = 0;
                return;
            }
        }
    }

    /**
     * @brief Removes all entries from the database and the position information too.
     *
     */
    void clearAll() {
        for (size_t i = 0; i < MAX_NODES; ++i) {
            valid[i] = false;
            nodeinfos[i].node_id = 0;
            is_position_valid[i] = false;
        }
    }

    /**
     * @brief Get the Position object for the node with the given node_id.
     *
     * @param node_id
     * @param out_position
     * @return true Got position information for the node.
     * @return false Don't have position information for the node.
     */
    bool getPosition(uint32_t node_id, MC_Position& out_position) const {
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i].node_id == node_id && is_position_valid[i]) {
                out_position = positions[i];
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Set the Position object for the node with the given node_id.
     *
     * @param node_id
     * @param position
     * @return true Successfully set the position for the node.
     * @return false Node with the given node_id not found or position not set.
     */
    bool setPosition(uint32_t node_id, const MC_Position& position) {
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i].node_id == node_id) {
                positions[i] = position;
                is_position_valid[i] = true;
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Remove the position information for the node with the given node_id.
     *
     * @param node_id
     */
    void removePosition(uint32_t node_id) {
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i].node_id == node_id) {
                is_position_valid[i] = false;
                // Optionally clear position data:
                positions[i] = MC_Position{};
                return;
            }
        }
    }

   private:
    MC_NodeInfo nodeinfos[MAX_NODES];
    MC_Position positions[MAX_NODES];
    bool is_position_valid[MAX_NODES] = {};  // stores if the position is stored for the node
    bool valid[MAX_NODES] = {};              // stores if the index is taken or free
};

class MeshCompactOutQueue {
   public:
    static constexpr size_t MAX_ENTRIES = 15;

    // Add entry to queue, returns true if successful, false if full
    bool push(const MC_OutQueueEntry& entry, bool priority = false) {
        std::unique_lock<std::mutex> lock(mtx);
        if (queue.size() >= MAX_ENTRIES && !priority) {
            return false;
        }
        if (!priority) {
            queue.push_back(entry);
        } else {
            queue.push_front(entry);
            while (queue.size() > MAX_ENTRIES) {
                queue.pop_back();
            }
        }
        cv.notify_one();
        return true;
    }

    // Remove and get entry from queue, blocks if empty
    MC_OutQueueEntry pop() {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return (!queue.empty() || stopFlag); });
        MC_OutQueueEntry entry;
        entry.header.srcnode = 0;
        if (stopFlag) {
            // Return a default-constructed (empty) entry if stopFlag is set
            return entry;
        }
        entry = queue.front();
        queue.pop_front();
        return entry;
    }

    // Notify one waiting thread to unblock pop() immediately (even if queue is empty)
    void stop_wait() {
        stopFlag = true;
        cv.notify_one();
    }

    // Try to remove and get entry from queue, returns true if successful
    bool try_pop(MC_OutQueueEntry& entry) {
        std::unique_lock<std::mutex> lock(mtx);
        if (queue.empty()) return false;
        entry = queue.front();
        queue.pop_front();
        return true;
    }

    // Check if queue is empty
    bool empty() const {
        std::lock_guard<std::mutex> lock(mtx);
        return queue.empty();
    }

    // Get current size
    size_t size() const {
        std::lock_guard<std::mutex> lock(mtx);
        return queue.size();
    }

   private:
    mutable std::mutex mtx;
    std::condition_variable cv;
    std::deque<MC_OutQueueEntry> queue;
    bool stopFlag = false;
};

class MeshtasticCompactRouter {
   public:
    static constexpr size_t MAX_ENTRIES = 20;

    struct Entry {
        uint32_t src;
        uint32_t msgid;
    };

    void setExcludeSelf(bool value) { exclude_self = value; }  // Please don't adjust this unless you know what you are doing!
    void setMyId(uint32_t id) { my_id = id; }

    MeshtasticCompactRouter() : count(0), head(0) {}

    // Returns true if (src, msgid) was not present and is now inserted, false if already present
    bool onCheck(uint32_t src, uint32_t msgid) {
        if (exclude_self && src == my_id) {
            // ESP_LOGI("Router", "Ignoring message from self: src=%" PRIu32 ", msgid=%" PRIu32, src, msgid);
            return false;  // Ignore messages from self
        }
        std::lock_guard<std::mutex> lock(mtx);
        // Check for existence
        for (size_t i = 0; i < count; ++i) {
            size_t idx = (head - 1 - i + MAX_ENTRIES) % MAX_ENTRIES;
            if (entries[idx].src == src && entries[idx].msgid == msgid) {
                // ESP_LOGI("Router", "Ignoring message duplicated: src=%" PRIu32 ", msgid=%" PRIu32, src, msgid);
                return false;
            }
        }

        // Insert new entry at head (LRU: overwrite oldest if full)
        entries[head] = {src, msgid};
        head = (head + 1) % MAX_ENTRIES;
        if (count < MAX_ENTRIES) ++count;
        return true;
    }

   private:
    Entry entries[MAX_ENTRIES];
    size_t count;
    size_t head;               // Points to next insert position (oldest overwritten)
    bool exclude_self = true;  // Exclude self messages by default
    uint32_t my_id = 0;        // My node ID, used to exclude
    std::mutex mtx;
};

class MeshtasticCompact {
   public:
    MeshtasticCompact();
    ~MeshtasticCompact();
    bool RadioInit(RadioType radio_type, Radio_PINS& radio_pins, LoraConfig& lora_config);  // Initializes the radio with the given configuration and pins

    // callbacks
    using OnMessageCallback = void (*)(MC_Header& header, MC_TextMessage& message);
    using OnPositionMessageCallback = void (*)(MC_Header& header, MC_Position& position, bool needReply);
    using OnNodeInfoCallback = void (*)(MC_Header& header, MC_NodeInfo& nodeinfo, bool needReply);
    using OnWaypointMessageCallback = void (*)(MC_Header& header, MC_Waypoint& waypoint);
    using OnTelemetryDeviceCallback = void (*)(MC_Header& header, MC_Telemetry_Device& telemetry);
    using OnTelemetryEnvironmentCallback = void (*)(MC_Header& header, MC_Telemetry_Environment& telemetry);
    using OnTracerouteCallback = void (*)(MC_Header& header, MC_RouteDiscovery& route, bool for_me, bool is_reply, bool need_reply);
    using OnRaw = void (*)(const uint8_t* data, size_t len);
    void setOnWaypointMessage(OnWaypointMessageCallback cb) { onWaypointMessage = cb; }
    void setOnNodeInfoMessage(OnNodeInfoCallback cb) { onNodeInfo = cb; }
    void setOnPositionMessage(OnPositionMessageCallback cb) { onPositionMessage = cb; }
    void setOnMessage(OnMessageCallback cb) { onMessage = cb; }
    void setOnTelemetryDevice(OnTelemetryDeviceCallback cb) { onTelemetryDevice = cb; }
    void setOnTelemetryEnvironment(OnTelemetryEnvironmentCallback cb) { onTelemetryEnvironment = cb; }
    void setOnTraceroute(OnTracerouteCallback cb) { onTraceroute = cb; }
    void setOnRaw(OnRaw cb) { onRaw = cb; }
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

    NodeInfoDB nodeinfo_db;          // NodeInfo database.
    MeshtasticCompactRouter router;  // Router for message deduplication. Set MyId if you changed that. Also you can disable exclude self option
    MC_Position my_position;         // My position, used for auto replies (when enabled) on position requests. Or when you call SendMyPosition()
   private:
    bool RadioListen();    // inits the listening thread for the radio
    bool RadioSendInit();  // inits the sending thread for the radio. consumes the out_queue
    // handlers
    void intOnMessage(MC_Header& header, MC_TextMessage& message);                           // Called when got any text based messages
    void intOnPositionMessage(MC_Header& header, MC_Position& position, bool want_reply);    // Called on position messages
    void intOnNodeInfo(MC_Header& header, MC_NodeInfo& nodeinfo, bool want_reply);           // Called on node info messages
    void intOnWaypointMessage(MC_Header& header, MC_Waypoint& waypoint);                     // Called on waypoint messages
    void intOnTelemetryDevice(MC_Header& header, MC_Telemetry_Device& telemetry);            // Called on telemetry device messages
    void intOnTelemetryEnvironment(MC_Header& header, MC_Telemetry_Environment& telemetry);  // Called on telemetry environment messages
    void intOnTraceroute(MC_Header& header, MC_RouteDiscovery& route_discovery);             // Called on traceroute messages

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
};

/**
 * @brief Simple static helpers for building message objects easily.
 *
 */
class MeshtasticCompactHelpers {
   public:
    static void NodeInfoBuilder(MC_NodeInfo* nodeinfo, uint32_t node_id, std::string& short_name, std::string& long_name, uint8_t hw_model);
    static void PositionBuilder(MC_Position& position, float latitude, float longitude, int32_t altitude = 0, uint32_t speed = 0, uint32_t sats_in_view = 0);
    static void TelemetryDeviceBuilder(MC_Telemetry_Device& telemetry, uint32_t uptime_seconds = 0, float voltage = 0.0f, float battery_level = -1.0f, float channel_utilization = -1.0f);
    static void TelemetryEnvironmentBuilder(MC_Telemetry_Environment& telemetry, float temperature = -10000.0f, float humidity = -1.0f, float pressure = -1.0f, float lux = -1.0f);
    static void WaypointBuilder(MC_Waypoint& waypoint, uint32_t id, float latitude, float longitude, std::string name, std::string description, uint32_t expire = 1, uint32_t icon = 0);
};

#endif  // MESHTASTIC_COMPACT_H