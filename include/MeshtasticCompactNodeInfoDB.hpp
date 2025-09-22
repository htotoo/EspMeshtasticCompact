#pragma once

#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <string.h>
#include "MeshtasticCompactStructs.hpp"
#include "esp_random.h"
#include <vector>
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
        iterator(MCT_NodeInfo* nodeinfos, bool* valid, size_t idx)
            : nodeinfos_(nodeinfos), valid_(valid), idx_(idx) {
            advance_to_valid();
        }
        iterator& operator++() {
            ++idx_;
            advance_to_valid();
            return *this;
        }
        MCT_NodeInfo& operator*() {
            assert(idx_ < MAX_NODES && "Attempted to dereference an end() or invalid iterator");
            return nodeinfos_[idx_];
        }
        MCT_NodeInfo* operator->() {
            assert(idx_ < MAX_NODES && "Attempted to dereference an end() or invalid iterator");
            return &nodeinfos_[idx_];
        }
        bool operator!=(const iterator& other) const { return idx_ != other.idx_; }
        bool operator==(const iterator& other) const { return idx_ == other.idx_; }

       private:
        void advance_to_valid() {
            while (idx_ < MAX_NODES && !valid_[idx_]) ++idx_;
        }
        MCT_NodeInfo* nodeinfos_;
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
     * @return MCT_NodeInfo*
     */
    MCT_NodeInfo* getByIndex(size_t index) {
        if (index < MAX_NODES && valid[index]) {
            return &nodeinfos[index];
        }
        return nullptr;
    }

    /**
     * @brief Get the Random Node object
     *
     * @return MCT_NodeInfo*
     */
    MCT_NodeInfo* getRandomNode() {
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
     * @return true When new node inserted.
     */
    bool addOrUpdate(uint32_t node_id, const MCT_NodeInfo& info) {
        // Try to update existing
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i].node_id == node_id) {
                nodeinfos[i] = info;
                return false;
            }
        }
        // Add new if space
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (!valid[i]) {
                nodeinfos[i] = info;
                valid[i] = true;
                is_position_valid[i] = false;
                positions[i].has_latitude_i = false;
                positions[i].has_longitude_i = false;
                return true;
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
        positions[oldest_idx].has_latitude_i = false;
        positions[oldest_idx].has_longitude_i = false;
        return true;
    }

    /**
     * @brief Returns a pointer to the node information for the given node_id.
     *
     * @param node_id
     * @return MCT_NodeInfo*
     */
    MCT_NodeInfo* get(uint32_t node_id) {
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
    bool getPosition(uint32_t node_id, MCT_Position& out_position) const {
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
    bool setPosition(uint32_t node_id, const MCT_Position& position) {
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
                positions[i] = MCT_Position{};
                return;
            }
        }
    }

    bool serialize(std::vector<uint8_t>& data) const {
        // Save valid array
        data.insert(data.end(), reinterpret_cast<const uint8_t*>(valid), reinterpret_cast<const uint8_t*>(valid) + sizeof(valid));
        // Save nodeinfos array
        data.insert(data.end(), reinterpret_cast<const uint8_t*>(nodeinfos), reinterpret_cast<const uint8_t*>(nodeinfos) + sizeof(nodeinfos));
        // Save is_position_valid array
        data.insert(data.end(), reinterpret_cast<const uint8_t*>(is_position_valid), reinterpret_cast<const uint8_t*>(is_position_valid) + sizeof(is_position_valid));
        // Save positions array
        data.insert(data.end(), reinterpret_cast<const uint8_t*>(positions), reinterpret_cast<const uint8_t*>(positions) + sizeof(positions));
        return true;
    }

    bool deserialize(const std::vector<uint8_t>& data) {
        size_t expected_size = sizeof(valid) + sizeof(nodeinfos) + sizeof(is_position_valid) + sizeof(positions);
        if (data.size() != expected_size) return false;
        size_t offset = 0;
        // Restore valid array
        memcpy(valid, data.data() + offset, sizeof(valid));
        offset += sizeof(valid);
        // Restore nodeinfos array
        memcpy(nodeinfos, data.data() + offset, sizeof(nodeinfos));
        offset += sizeof(nodeinfos);
        // Restore is_position_valid array
        memcpy(is_position_valid, data.data() + offset, sizeof(is_position_valid));
        offset += sizeof(is_position_valid);
        // Restore positions array
        memcpy(positions, data.data() + offset, sizeof(positions));
        return true;
    }

   private:
    MCT_NodeInfo nodeinfos[MAX_NODES];
    MCT_Position positions[MAX_NODES];
    bool is_position_valid[MAX_NODES] = {};  // stores if the position is stored for the node
    bool valid[MAX_NODES] = {};              // stores if the index is taken or free
};
