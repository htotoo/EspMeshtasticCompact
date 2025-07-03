#ifndef MESHTASTIC_COMPACT_H
#define MESHTASTIC_COMPACT_H

#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <string.h>
#include "pb.h"
#include <RadioLib.h>
#include "EspHal.h"

// https://github.com/meshtastic/firmware/blob/81828c6244daede254cf759a0f2bd939b2e7dd65/variants/heltec_wsl_v3/variant.h

class MeshtasticCompact {
   public:
    MeshtasticCompact();
    ~MeshtasticCompact();

    bool RadioInit();
    bool RadioListen();
    bool DebugPacket(uint8_t* data, int len);

   private:
    bool pb_decode_from_bytes(const uint8_t* srcbuf, size_t srcbufsize, const pb_msgdesc_t* fields, void* dest_struct);
    static void task_listen(void* pvParameters);
    void generate_meshtastic_nonce(uint32_t packet_id, uint32_t from_node, uint8_t* nonce);
    bool decrypt_meshtastic_payload(const uint8_t* key, uint32_t packet_id, uint32_t from_node, const uint8_t* encrypted_in, uint8_t* decrypted_out, size_t len);

    EspHal* hal = new EspHal(9, 11, 10);
    SX1262 radio = new Module(hal, 8, 14, 12, 13);

    const uint8_t default_channel_key[32] = {
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
};

#endif  // MESHTASTIC_COMPACT_H