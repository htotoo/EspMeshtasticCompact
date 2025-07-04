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
// https://github.com/meshtastic/firmware/blob/81828c6244daede254cf759a0f2bd939b2e7dd65/variants/heltec_wsl_v3/variant.h

class MeshtasticCompact {
   public:
    MeshtasticCompact();
    ~MeshtasticCompact();

    bool RadioInit();

    using OnMessageCallback = void (*)(uint8_t chan, std::string message, uint32_t srcnode, uint32_t dstnode, uint8_t flag);  // flag: 0 normal text, 1 = alert, 3 = detector sensor, 4 ping, 5 uart, 6 range test
    void setOnMessage(OnMessageCallback cb) { onMessage = cb; }
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

   private:
    bool RadioListen();
    // handlers
    void intOnMessage(uint8_t chan, std::string message, uint32_t srcnode, uint32_t dstnode, uint8_t flag = 0);

    // mesh network minimum functionality
    bool send_ack();

    // decoding
    int16_t ProcessPacket(uint8_t* data, int len);
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
};

#endif  // MESHTASTIC_COMPACT_H