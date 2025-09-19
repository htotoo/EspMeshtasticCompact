#pragma once

#include <stdint.h>
#include <mutex>

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