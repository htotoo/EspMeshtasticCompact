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
#include "MeshasticCompactStructs.hpp"

// https://github.com/meshtastic/firmware/blob/81828c6244daede254cf759a0f2bd939b2e7dd65/variants/heltec_wsl_v3/variant.h

class NodeInfoDB {
   public:
    static constexpr size_t MAX_NODES = 32;

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
                return;
            }
        }
        // No space: do nothing (could implement LRU or overwrite oldest if needed)
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

class MeshtasticCompact {
   public:
    MeshtasticCompact();
    ~MeshtasticCompact();

    bool RadioInit();

    // callbacks
    using OnMessageCallback = void (*)(MC_Header header, MC_TextMessage message);
    using OnPositionMessageCallback = void (*)(MC_Header header, MC_Position position);
    using OnNodeInfoCallback = void (*)(MC_Header header, MC_NodeInfo nodeinfo);
    using OnWaypointMessageCallback = void (*)(MC_Header header, MC_Waypoint waypoint);
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
    bool setSendHopLimit(uint8_t limit) {
        if (limit > 0 && limit <= 7) {
            send_hop_limit = limit;
            return true;
        }
        return false;
    }

    NodeInfoDB nodeinfo_db;  // NodeInfo database

   private:
    bool RadioListen();
    // handlers
    void intOnMessage(MC_Header header, MC_TextMessage message);
    void intOnPositionMessage(MC_Header header, MC_Position position);
    void intOnNodeInfo(MC_Header header, MC_NodeInfo nodeinfo);
    void intOnWaypointMessage(MC_Header header, MC_Waypoint waypoint);
    // mesh network minimum functionality
    bool send_ack();

    // decoding
    int16_t ProcessPacket(uint8_t* data, int len, MeshtasticCompact* mshcomp);
    int16_t try_decode_root_packet(const uint8_t* srcbuf, size_t srcbufsize, const pb_msgdesc_t* fields, void* dest_struct, size_t dest_struct_size, uint32_t packet_id, uint32_t packet_src);
    bool pb_decode_from_bytes(const uint8_t* srcbuf, size_t srcbufsize, const pb_msgdesc_t* fields, void* dest_struct);
    size_t pb_encode_to_bytes(uint8_t* destbuf, size_t destbufsize, const pb_msgdesc_t* fields, const void* src_struct);
    static void task_listen(void* pvParameters);
    bool aes_decrypt_meshtastic_payload(const uint8_t* key, uint16_t keySize, uint32_t packet_id, uint32_t from_node, const uint8_t* encrypted_in, uint8_t* decrypted_out, size_t len);

    float rssi, snr;
    bool is_send_enabled = true;
    uint8_t send_hop_limit = 7;  // default hop limit for sending packets
    uint32_t my_id = 0;

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

    // Function pointer for onMessage callback
    OnMessageCallback onMessage = nullptr;
    OnPositionMessageCallback onPositionMessage = nullptr;
    OnNodeInfoCallback onNodeInfo = nullptr;
    OnWaypointMessageCallback onWaypointMessage = nullptr;
};

#endif  // MESHTASTIC_COMPACT_H