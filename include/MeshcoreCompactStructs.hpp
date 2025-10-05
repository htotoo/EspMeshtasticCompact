#ifndef MeshcoreCompactStructs_h
#define MeshcoreCompactStructs_h

#include <stdint.h>
#include <string>
#include <vector>
#include "RadioStructs.hpp"
#include <cstring>

enum class MCC_ROUTE_TYPE {
    ROUTE_TYPE_TRANSPORT_FLOOD = 0,
    ROUTE_TYPE_FLOOD = 1,
    ROUTE_TYPE_DIRECT = 2,
    ROUTE_TYPE_TRANSPORT_DIRECT = 3
};

enum class MCC_PAYLOAD_TYPE : uint8_t {
    PAYLOAD_TYPE_REQ = 0x00,
    PAYLOAD_TYPE_RESPONSE = 0x01,
    PAYLOAD_TYPE_TXT_MSG = 0x02,
    PAYLOAD_TYPE_ACK = 0x03,
    PAYLOAD_TYPE_ADVERT = 0x04,
    PAYLOAD_TYPE_GRP_TXT = 0x05,
    PAYLOAD_TYPE_GRP_DATA = 0x06,
    PAYLOAD_TYPE_ANON_REQ = 0x07,
    PAYLOAD_TYPE_PATH = 0x08,
    PAYLOAD_TYPE_TRACE = 0x09,
    PAYLOAD_TYPE_MULTIPART = 0x0A,
    PAYLOAD_TYPE_RAW_CUSTOM = 0x0F
};

enum class MCC_ADDR_FORMAT : uint8_t {
    ADDR_FORMAT_1 = 0x00,           // 1-byte src/dest hashes, 2-byte MAC
    ADDR_FORMAT_RESERVED_1 = 0x01,  // Future version: 2-byte hashes, 4-byte MAC
    ADDR_FORMAT_RESERVED_2 = 0x02,  // Future version
    ADDR_FORMAT_RESERVED_3 = 0x03   // Future version
};

enum class MCC_NODEINFO_FLAGS : uint8_t {
    IS_CHAT_NODE = 0x01,    // advert is for a chat node
    IS_REPEATER = 0x02,     // advert is for a repeater
    IS_ROOM_SERVER = 0x03,  // advert is for a room server
    IS_SENSOR = 0x04,       // advert is for a sensor server
    HAS_LOCATION = 0x10,    // appdata contains lat/long information
    HAS_FEATURE_1 = 0x20,   // Reserved for future use
    HAS_FEATURE_2 = 0x40,   // Reserved for future use
    HAS_NAME = 0x80         // appdata contains a node name
};

class MCC_HEADER {
   public:
    MCC_ROUTE_TYPE get_route_type() {
        return (MCC_ROUTE_TYPE)(header & 0x03);
    }
    MCC_PAYLOAD_TYPE get_payload_type() {
        return (MCC_PAYLOAD_TYPE)((header >> 2) & 0x0F);
    }
    MCC_ADDR_FORMAT get_addr_format() {
        return (MCC_ADDR_FORMAT)((header >> 6) & 0x03);
    }

    /**
     * @brief Parse the MCC header from the given data buffer.
     *
     * @param data Pointer to the data buffer.
     * @param len Length of the data buffer.
     * @return size_t Number of bytes consumed from the buffer, or 0 on failure.
     */

    size_t parse(uint8_t* data, size_t len) {
        if (len < 4) return 0;  // todo better solution, and further checks
        size_t pos = 0;
        header = data[pos++];
        if (get_route_type() == MCC_ROUTE_TYPE::ROUTE_TYPE_TRANSPORT_FLOOD || get_route_type() == MCC_ROUTE_TYPE::ROUTE_TYPE_TRANSPORT_DIRECT) {
            // in this case we have transport code 4 bytes
            transport_codes = *((uint32_t*)&data[pos]);
            pos += 4;
        } else {
            transport_codes = 0;
        }
        uint8_t path_length = data[pos++];
        path.resize(path_length);
        // if (len < pos + path_length) return 0;
        memcpy(path.data(), &data[pos], path_length);
        pos += path_length;
        return pos;
    }

    uint8_t header;
    uint32_t transport_codes;   // optional 4 bytes
    std::vector<uint8_t> path;  // it'll store path_len too
};

class MCC_NODEINFO {
   public:
    size_t parse(const uint8_t* data, size_t start_pos, size_t len) {
        if (len < 36 + start_pos) return 0;  // minimum size
        size_t pos = start_pos;
        memcpy(pubkey, &data[pos], 32);
        pos += 32;
        timestamp = *((uint32_t*)&data[pos]);
        pos += 4;
        // skip signature
        pos += 64;
        flags = data[pos++];
        if (flags & (uint8_t)MCC_NODEINFO_FLAGS::HAS_LOCATION) {
            if (len < pos + 7) return 0;
            latitude_i = *((uint32_t*)&data[pos]);
            pos += 4;
            longitude_i = *((uint32_t*)&data[pos]);
            pos += 4;
        }
        if (flags & (uint8_t)MCC_NODEINFO_FLAGS::HAS_FEATURE_1) {
            // skip feature 1 data for now
            pos += 2;
        }
        if (flags & (uint8_t)MCC_NODEINFO_FLAGS::HAS_FEATURE_2) {
            // skip feature 2 data for now
            pos += 2;
        }
        if (flags & (uint8_t)MCC_NODEINFO_FLAGS::HAS_NAME) {
            uint8_t name_len = len - pos;
            name = std::string((const char*)&data[pos], name_len);
            pos += name_len;
        }
        return pos;
    }
    uint8_t pubkey[32];
    uint32_t timestamp;
    uint8_t flags;
    uint32_t latitude_i;   // optional
    uint32_t longitude_i;  // optional
    std::string name;
};

#endif  // MeshCoreCompactStructs_h