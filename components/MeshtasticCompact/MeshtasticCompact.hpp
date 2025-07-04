#ifndef MESHTASTIC_COMPACT_H
#define MESHTASTIC_COMPACT_H

#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <string.h>
#include "pb.h"
#include <RadioLib.h>
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

    // Add or update a node entry
    void addOrUpdate(uint32_t node_id, const MC_NodeInfo& info) {
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

    // Get pointer to nodeinfo by node_id, or nullptr if not found
    MC_NodeInfo* get(uint32_t node_id) {
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i].node_id == node_id) {
                return &nodeinfos[i];
            }
        }
        return nullptr;
    }

    // Delete a node entry by node_id
    void remove(uint32_t node_id) {
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i].node_id == node_id) {
                valid[i] = false;
                nodeinfos[i].node_id = 0;
                return;
            }
        }
    }

    // Clear all node entries
    void clearAll() {
        for (size_t i = 0; i < MAX_NODES; ++i) {
            valid[i] = false;
            nodeinfos[i].node_id = 0;
            is_position_valid[i] = false;
        }
    }

    // Get position for a valid node by node_id, returns true if found and valid
    bool getPosition(uint32_t node_id, MC_Position& out_position) const {
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i].node_id == node_id && is_position_valid[i]) {
                out_position = positions[i];
                return true;
            }
        }
        return false;
    }

    // Set position for a valid node by node_id, returns true if successful
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

    // Remove position for a node by node_id
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
    MC_Position positions[MAX_NODES];  // Optional: if you want to store positions too
    bool is_position_valid[MAX_NODES] = {};
    bool valid[MAX_NODES] = {};
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

    void setExcludeSelf(bool value) { exclude_self = value; }
    void setMyId(uint32_t id) { my_id = id; }

    MeshtasticCompactRouter() : count(0), head(0) {}

    // Returns true if (src, msgid) was not present and is now inserted, false if already present
    bool onCheck(uint32_t src, uint32_t msgid) {
        if (exclude_self && src == my_id) {
            ESP_LOGI("Router", "Ignoring message from self: src=%" PRIu32 ", msgid=%" PRIu32, src, msgid);
            return false;  // Ignore messages from self
        }
        std::lock_guard<std::mutex> lock(mtx);
        // Check for existence
        for (size_t i = 0; i < count; ++i) {
            size_t idx = (head - 1 - i + MAX_ENTRIES) % MAX_ENTRIES;
            if (entries[idx].src == src && entries[idx].msgid == msgid) {
                ESP_LOGI("Router", "Ignoring message duplicated: src=%" PRIu32 ", msgid=%" PRIu32, src, msgid);
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
    bool RadioInit();

    // callbacks
    using OnMessageCallback = void (*)(MC_Header header, MC_TextMessage& message);
    using OnPositionMessageCallback = void (*)(MC_Header header, MC_Position& position, bool needReply);
    using OnNodeInfoCallback = void (*)(MC_Header header, MC_NodeInfo& nodeinfo, bool needReply);
    using OnWaypointMessageCallback = void (*)(MC_Header header, MC_Waypoint& waypoint);
    void setOnWaypointMessage(OnWaypointMessageCallback cb) { onWaypointMessage = cb; }
    void setOnNodeInfoMessage(OnNodeInfoCallback cb) { onNodeInfo = cb; }
    void setOnPositionMessage(OnPositionMessageCallback cb) { onPositionMessage = cb; }
    void setOnMessage(OnMessageCallback cb) { onMessage = cb; }

    //
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

    void setMyNames(const char* short_name, const char* long_name);
    MC_NodeInfo* getMyNodeInfo() {
        return &my_nodeinfo;
    }

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
    NodeInfoDB nodeinfo_db;          // NodeInfo database
    MeshtasticCompactRouter router;  // Router for message deduplication. Set MyId if you changed that. Also you can disable exclude self option
    MC_Position my_position;         // My position, used for replies
   private:
    bool RadioListen();
    bool RadioSendInit();
    // handlers
    void intOnMessage(MC_Header header, MC_TextMessage message);
    void intOnPositionMessage(MC_Header header, MC_Position position, bool want_reply);
    void intOnNodeInfo(MC_Header header, MC_NodeInfo nodeinfo, bool want_reply);
    void intOnWaypointMessage(MC_Header header, MC_Waypoint waypoint);
    // mesh network minimum functionality
    bool send_ack();

    // decoding
    int16_t ProcessPacket(uint8_t* data, int len, MeshtasticCompact* mshcomp);
    int16_t try_decode_root_packet(const uint8_t* srcbuf, size_t srcbufsize, const pb_msgdesc_t* fields, void* dest_struct, size_t dest_struct_size, MC_Header& header);
    bool pb_decode_from_bytes(const uint8_t* srcbuf, size_t srcbufsize, const pb_msgdesc_t* fields, void* dest_struct);
    size_t pb_encode_to_bytes(uint8_t* destbuf, size_t destbufsize, const pb_msgdesc_t* fields, const void* src_struct);
    static void task_listen(void* pvParameters);
    static void task_send(void* pvParameters);
    bool aes_decrypt_meshtastic_payload(const uint8_t* key, uint16_t keySize, uint32_t packet_id, uint32_t from_node, const uint8_t* encrypted_in, uint8_t* decrypted_out, size_t len);

    float rssi, snr;
    bool is_send_enabled = true;
    uint8_t send_hop_limit = 7;  // default hop limit for sending packets
    bool is_auto_full_node = true;

    MC_NodeInfo my_nodeinfo;  // My node info

    EspHal* hal = new EspHal(9, 11, 10);
    SX1262 radio = new Module(hal, 8, 14, 12, 13);

    const uint8_t default_l1_key[16] =
        {0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
         0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01};
    const uint8_t default_chan_key[32] = {
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    mbedtls_aes_context aes_ctx;
    mutable std::mutex mtx;
    bool need_run = true;

    MeshCompactOutQueue out_queue;

    // Function pointer for onMessage callback
    OnMessageCallback onMessage = nullptr;
    OnPositionMessageCallback onPositionMessage = nullptr;
    OnNodeInfoCallback onNodeInfo = nullptr;
    OnWaypointMessageCallback onWaypointMessage = nullptr;
};

class MeshtasticCompactHelpers {
   public:
    static void NodeInfoBuilder(MC_NodeInfo& nodeinfo, uint32_t node_id, std::string& short_name, std::string& long_name);
    static void PositionBuilder(MC_Position& position, float latitude, float longitude, int32_t altitude = 0, uint32_t speed = 0, uint32_t sats_in_view = 0);
};

#endif  // MESHTASTIC_COMPACT_H