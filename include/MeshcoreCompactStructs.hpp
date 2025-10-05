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
        if (get_route_type() == MCC_ROUTE_TYPE::ROUTE_TYPE_TRANSPORT_FLOOD || get_route_type() == MCC_ROUTE_TYPE::ROUTE_TYPE_FLOOD) {
            // in this case we have transport code 4 bytes
            transport_codes = *((uint32_t*)&data[pos]);
            pos += 4;
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

#endif  // MeshCoreCompactStructs_h