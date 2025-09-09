#ifndef ESP_HAL_H
#define ESP_HAL_H

// include RadioLib
#include <RadioLib.h>

// include all the dependencies
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc.h"
#include "soc/dport_reg.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/spi_master.h"

// define Arduino-style macros
#define LOW (0x0)
#define HIGH (0x1)
#define INPUT (0x01)
#define OUTPUT (0x03)
#define RISING (0x01)
#define FALLING (0x02)
#define NOP() asm volatile("nop")

#define MATRIX_DETACH_OUT_SIG (0x100)
#define MATRIX_DETACH_IN_LOW_PIN (0x30)

// all of the following is needed to calculate SPI clock divider
// #define ClkRegToFreq(reg)(apb_freq / (((reg)->clkdiv_pre + 1) * ((reg)->clkcnt_n + 1)))
/*
typedef union {
    uint32_t value;
    struct {
        uint32_t clkcnt_l : 6;
        uint32_t clkcnt_h : 6;
        uint32_t clkcnt_n : 6;
        uint32_t clkdiv_pre : 13;
        uint32_t clk_equ_sysclk : 1;
    };
} spiClk_t;

uint32_t getApbFrequency() {
    rtc_cpu_freq_config_t conf;
    rtc_clk_cpu_freq_get_config(&conf);

    if (conf.freq_mhz >= 80) {
        return (80 * MHZ);
    }

    return ((conf.source_freq_mhz * MHZ) / conf.div);
}

uint32_t spiFrequencyToClockDiv(uint32_t freq) {
    uint32_t apb_freq = getApbFrequency();
    if (freq >= apb_freq) {
        return SPI_CLK_EQU_SYSCLK;
    }

    const spiClk_t minFreqReg = {0x7FFFF000};
    uint32_t minFreq = ClkRegToFreq((spiClk_t*)&minFreqReg);
    if (freq < minFreq) {
        return minFreqReg.value;
    }

    uint8_t calN = 1;
    spiClk_t bestReg = {0};
    int32_t bestFreq = 0;
    while (calN <= 0x3F) {
        spiClk_t reg = {0};
        int32_t calFreq;
        int32_t calPre;
        int8_t calPreVari = -2;

        reg.clkcnt_n = calN;

        while (calPreVari++ <= 1) {
            calPre = (((apb_freq / (reg.clkcnt_n + 1)) / freq) - 1) + calPreVari;
            if (calPre > 0x1FFF) {
                reg.clkdiv_pre = 0x1FFF;
            } else if (calPre <= 0) {
                reg.clkdiv_pre = 0;
            } else {
                reg.clkdiv_pre = calPre;
            }
            reg.clkcnt_l = ((reg.clkcnt_n + 1) / 2);
            calFreq = ClkRegToFreq(&reg);
            if (calFreq == (int32_t)freq) {
                memcpy(&bestReg, &reg, sizeof(bestReg));
                break;
            } else if (calFreq < (int32_t)freq) {
                if (RADIOLIB_ABS(freq - calFreq) < RADIOLIB_ABS(freq - bestFreq)) {
                    bestFreq = calFreq;
                    memcpy(&bestReg, &reg, sizeof(bestReg));
                }
            }
        }
        if (calFreq == (int32_t)freq) {
            break;
        }
        calN++;
    }
    return (bestReg.value);
}
*/

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

// create a new ESP-IDF hardware abstraction layer
// the HAL must inherit from the base RadioLibHal class
// and implement all of its virtual methods
// this is pretty much just copied from Arduino ESP32 core
class EspHal : public RadioLibHal {
   public:
    // default constructor - initializes the base HAL and any needed private members
    EspHal(int8_t sck, int8_t miso, int8_t mosi, int8_t nss = 8)
        : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING),
          spiSCK(sck),
          spiMISO(miso),
          spiMOSI(mosi),
          spiNss(nss) {
    }

    virtual ~EspHal() {}

    void init() override {
        // we only need to init the SPI here
        spiBegin();
    }

    void term() override {
        // we only need to stop the SPI here
        spiEnd();
    }

    // GPIO-related methods (pinMode, digitalWrite etc.) should check
    // RADIOLIB_NC as an alias for non-connected pins
    void pinMode(uint32_t pin, uint32_t mode) override {
        if (pin == RADIOLIB_NC) {
            return;
        }
        gpio_hal_context_t gpiohal;
        gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);

        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = (gpio_mode_t)mode,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = (gpio_int_type_t)gpiohal.dev->pin[pin].int_type,
        };
        gpio_config(&conf);
    }

    void digitalWrite(uint32_t pin, uint32_t value) override {
        if (pin == RADIOLIB_NC) {
            return;
        }
        gpio_set_level((gpio_num_t)pin, value);
    }

    uint32_t digitalRead(uint32_t pin) override {
        if (pin == RADIOLIB_NC) {
            return (0);
        }
        return (gpio_get_level((gpio_num_t)pin));
    }

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
        if (interruptNum == RADIOLIB_NC) {
            return;
        }
        gpio_install_isr_service((int)ESP_INTR_FLAG_IRAM);
        gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)(mode & 0x7));

        // this uses function typecasting, which is not defined when the functions have different signatures
        // untested and might not work
        gpio_isr_handler_add((gpio_num_t)interruptNum, (void (*)(void*))interruptCb, NULL);
    }

    void detachInterrupt(uint32_t interruptNum) override {
        if (interruptNum == RADIOLIB_NC) {
            return;
        }

        gpio_isr_handler_remove((gpio_num_t)interruptNum);
        gpio_wakeup_disable((gpio_num_t)interruptNum);
        gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
    }

    void delay(unsigned long ms) override {
        vTaskDelay(ms / portTICK_PERIOD_MS);
    }

    void delayMicroseconds(unsigned long us) override {
        uint64_t m = (uint64_t)esp_timer_get_time();
        if (us) {
            uint64_t e = (m + us);
            if (m > e) {  // overflow
                while ((uint64_t)esp_timer_get_time() > e) {
                    NOP();
                }
            }
            while ((uint64_t)esp_timer_get_time() < e) {
                NOP();
            }
        }
    }

    unsigned long millis() override {
        return ((unsigned long)(esp_timer_get_time() / 1000ULL));
    }

    unsigned long micros() override {
        return ((unsigned long)(esp_timer_get_time()));
    }

    long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override {
        if (pin == RADIOLIB_NC) {
            return (0);
        }
        this->pinMode(pin, INPUT);
        uint32_t start = this->micros();
        uint32_t curtick = this->micros();

        while (this->digitalRead(pin) == state) {
            if ((this->micros() - curtick) > timeout) {
                return (0);
            }
        }
        return (this->micros() - start);
    }

    void spiBegin() {
        // gpio_reset_pin(SX126x_SPI_SELECT);
        // gpio_set_direction(SX126x_SPI_SELECT, GPIO_MODE_OUTPUT);
        // gpio_set_level(SX126x_SPI_SELECT, 1);

        // gpio_reset_pin(SX126x_RESET);
        // gpio_set_direction(SX126x_RESET, GPIO_MODE_OUTPUT);

        // gpio_reset_pin(SX126x_BUSY);
        // gpio_set_direction(SX126x_BUSY, GPIO_MODE_INPUT);

        /*if (SX126x_TXEN != -1) {
            gpio_reset_pin(SX126x_TXEN);
            gpio_set_direction(SX126x_TXEN, GPIO_MODE_OUTPUT);
        }

        if (SX126x_RXEN != -1) {
            gpio_reset_pin(SX126x_RXEN);
            gpio_set_direction(SX126x_RXEN, GPIO_MODE_OUTPUT);
        }*/

        spi_bus_config_t spi_bus_config = {
            .mosi_io_num = spiMOSI,
            .miso_io_num = spiMISO,
            .sclk_io_num = spiSCK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1};

        esp_err_t ret;
        ret = spi_bus_initialize(SPI2_HOST, &spi_bus_config, SPI_DMA_CH_AUTO);
        ESP_LOGI("ESPHal", "spi_bus_initialize=%d", ret);
        if (ret != 259) assert(ret == ESP_OK);  // 259 = already inited

        spi_device_interface_config_t devcfg = {
            .mode = 0,
            .clock_speed_hz = 9000000,
            .spics_io_num = spiNss,
            .flags = 0,
            .queue_size = 7,
            .pre_cb = NULL};
        // spi_device_handle_t handle;
        ret = spi_bus_add_device(SPI2_HOST, &devcfg, &SpiHandle);
        ESP_LOGI("ESPHal", "spi_bus_add_device=%d", ret);
    }

    void spiBeginTransaction() {
        // not needed - in ESP32 Arduino core, this function
        // repeats clock div, mode and bit order configuration
        spi_device_acquire_bus(SpiHandle, portMAX_DELAY);
    }

    uint8_t spiTransferByte(uint8_t b) {
        spi_transaction_t SPITransaction;
        uint8_t in = 0;
        memset(&SPITransaction, 0, sizeof(spi_transaction_t));
        SPITransaction.length = 8;
        SPITransaction.tx_buffer = &b;
        SPITransaction.rx_buffer = &in;
        if (spi_device_transmit(SpiHandle, &SPITransaction) == ESP_OK) {
            return in;
        }
        return 0;
    }

    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
        for (size_t i = 0; i < len; i++) {
            in[i] = this->spiTransferByte(out[i]);
        }
    }

    void spiEndTransaction() {
        // nothing needs to be done here
        spi_device_release_bus(SpiHandle);
    }

    void spiEnd() {
        spi_bus_remove_device(SpiHandle);
        // spi_bus_free(SPI2_HOST);
    }

   private:
    // the HAL can contain any additional private members
    int8_t spiSCK;
    int8_t spiMISO;
    int8_t spiMOSI;
    int8_t spiNss;
    spi_device_handle_t SpiHandle;
};
#pragma GCC diagnostic pop

#endif