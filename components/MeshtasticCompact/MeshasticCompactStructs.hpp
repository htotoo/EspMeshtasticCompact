#ifndef MeshtasticCompactStructs_h
#define MeshtasticCompactStructs_h

#include <stdint.h>

typedef enum {
    MC_MESSAGE_TYPE_TEXT = 0,             // Normal text message
    MC_MESSAGE_TYPE_ALERT = 1,            // Alert message
    MC_MESSAGE_TYPE_DETECTOR_SENSOR = 2,  // Detector sensor message
    MC_MESSAGE_TYPE_PING = 3,             // Ping message
    MC_MESSAGE_TYPE_UART = 4,             // UART message
    MC_MESSAGE_TYPE_RANGE_TEST = 5,       // Range test message
} MC_MESSAGE_TYPE;

struct MC_Header {
    uint32_t srcnode;  // source node ID
    uint32_t dstnode;  // destination node ID
    uint32_t packet_id;
    uint8_t hop_limit;
    uint8_t hop_start;
    uint8_t chan_hash;
    bool want_ack;  // true if the sender wants an acknowledgment
    bool via_mqtt;  // true if the packet is sent via MQTT
    float rssi;
    float snr;
};

struct MC_TextMessage {
    std::string text;
    uint8_t chan;
    MC_MESSAGE_TYPE type;
};

struct MC_Position {
    int32_t latitude_i;   // Latitude in degrees
    int32_t longitude_i;  // Longitude in degrees
    int32_t altitude;     // Altitude in meters
    uint32_t ground_speed;
    uint32_t sats_in_view;    // Number of satellites in view
    uint8_t location_source;  // Source of the location data (e.g., GPS, network)
};
#endif  // MeshtasticCompactStructs_h