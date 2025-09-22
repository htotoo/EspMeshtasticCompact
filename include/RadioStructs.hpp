#pragma once
#include <stdint.h>

enum class RadioType {
    SX1262,
    SX1261,
    SX1268,
    SX1276
};

struct Radio_PINS {
    uint8_t sck;
    uint8_t miso;
    uint8_t mosi;
    uint8_t cs;
    uint8_t irq;
    uint8_t rst;
    uint8_t gpio;
};

struct LoraConfig {
    float frequency;           // Frequency in MHz
    float bandwidth;           // Bandwidth in kHz
    uint8_t spreading_factor;  // Spreading factor (7-12)
    uint8_t coding_rate;       // Coding rate denominator (5-8)
    uint8_t sync_word;         // Sync word
    uint16_t preamble_length;  // Preamble length in symbols
    int8_t output_power;       // Output power in dBm
    float tcxo_voltage;        // TCXO voltage in volts
    bool use_regulator_ldo;    // Use LDO regulator (true) or DC-DC regulator (false)
};
