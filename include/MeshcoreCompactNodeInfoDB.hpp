#pragma once

#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <string.h>
#include "MeshcoreCompactStructs.hpp"
#include "esp_random.h"
#include <vector>

/**
 * @brief Handles an in-memory database of node information.
 *
 */
class NodeInfoCoreDB {
   public:
    static constexpr size_t MAX_NODES = 50;

    // Iterator for NodeInfoCoreDB
    class iterator {
       public:
        iterator(MCC_Nodeinfo* nodeinfos, bool* valid, size_t idx)
            : nodeinfos_(nodeinfos), valid_(valid), idx_(idx) {
            advance_to_valid();
        }
        iterator& operator++() {
            ++idx_;
            advance_to_valid();
            return *this;
        }
        MCC_Nodeinfo& operator*() {
            assert(idx_ < MAX_NODES && "Attempted to dereference an end() or invalid iterator");
            return nodeinfos_[idx_];
        }
        MCC_Nodeinfo* operator->() {
            assert(idx_ < MAX_NODES && "Attempted to dereference an end() or invalid iterator");
            return &nodeinfos_[idx_];
        }
        bool operator!=(const iterator& other) const { return idx_ != other.idx_; }
        bool operator==(const iterator& other) const { return idx_ == other.idx_; }

       private:
        void advance_to_valid() {
            while (idx_ < MAX_NODES && !valid_[idx_]) ++idx_;
        }
        MCC_Nodeinfo* nodeinfos_;
        bool* valid_;
        size_t idx_;
    };

    iterator begin() { return iterator(nodeinfos, valid, 0); }
    iterator end() { return iterator(nodeinfos, valid, MAX_NODES); }

    MCC_Nodeinfo* getByIndex(size_t index) {
        if (index < MAX_NODES && valid[index]) {
            return &nodeinfos[index];
        }
        return nullptr;
    }

    MCC_Nodeinfo* getRandomNode() {
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
     * @brief Adds or updates a node in the database.
     *
     * @param info The node information to add or update.
     * @return true if a new node was added, false if an existing node was updated.
     */
    bool addOrUpdate(const MCC_Nodeinfo& info) {
        // Try to update existing
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i] == info) {
                nodeinfos[i] = info;
                return false;
            }
        }
        // Add new if space
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (!valid[i]) {
                nodeinfos[i] = info;
                valid[i] = true;
                return true;
            }
        }
        // No space:  LRU (overwrite the oldest entry)
        size_t oldest_idx = 0;
        uint32_t oldest_time = nodeinfos[0].timestamp;
        for (size_t i = 1; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i].timestamp < oldest_time) {
                oldest_time = nodeinfos[i].timestamp;
                oldest_idx = i;
            }
        }
        nodeinfos[oldest_idx] = info;
        return true;
    }

    /**
     * @brief Removes a node from the database.
     *
     * @param info The node information to remove.
     */
    void remove(const MCC_Nodeinfo& info) {
        for (size_t i = 0; i < MAX_NODES; ++i) {
            if (valid[i] && nodeinfos[i] == info) {
                valid[i] = false;
                memset(nodeinfos[i].pubkey, 0, sizeof(nodeinfos[i].pubkey));
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
            memset(nodeinfos[i].pubkey, 0, sizeof(nodeinfos[i].pubkey));
        }
    }

    bool serialize(std::vector<uint8_t>& data) const {
        // need to serialize to. todo
        return true;
    }

    bool deserialize(const std::vector<uint8_t>& data) {
        // need to deserialize from. todo

        return true;
    }

   private:
    MCC_Nodeinfo nodeinfos[MAX_NODES];
    bool valid[MAX_NODES] = {};  // stores if the index is taken or free
};
