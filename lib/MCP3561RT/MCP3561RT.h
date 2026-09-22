#pragma once
#include <Arduino.h>
#include <SPI.h>

// MCP3561RT 24-bit delta-sigma ADC driver
// Datasheet: MCP3561RT.pdf — SPI mode 0,0, device address 0b01

class MCP3561RT {
public:
    enum class Mux : uint8_t {
        CH0      = 0x00,
        CH1      = 0x01,
        CH2      = 0x02,
        CH3      = 0x03,
        AGND     = 0x08,
        AVDD     = 0x09,
        REFIN_P  = 0x0B,
        REFIN_N  = 0x0C,
        TEMP_P   = 0x0E,
        TEMP_N   = 0x0F
    };

    enum class Gain : uint8_t {
        X1  = 0b001,
        X2  = 0b010,
        X4  = 0b011,
        X8  = 0b100,
        X16 = 0b101
    };

    enum class OSR : uint8_t {
        OSR_32   = 0b0000,
        OSR_64   = 0b0001,
        OSR_128  = 0b0010,
        OSR_256  = 0b0011,
        OSR_512  = 0b0100,
        OSR_1024 = 0b0101
    };

    MCP3561RT(uint8_t csPin, uint8_t irqPin, SPIClass& bus, SPISettings settings, float vref);

    bool begin();
    void setMux(Mux vinp, Mux vinm);
    void setGain(Gain g);
    void setOSR(OSR osr);
    void trigger();
    bool dataReady() const;
    float read();
    bool readRaw(int32_t& out);
    uint8_t readRegister(uint8_t addr);

    // Reference voltage passed at construction — the scale factor for turning
    // raw counts into volts. Use this rather than a literal so each ADC's
    // conversions follow its own configured reference.
    float vref() const { return _vref; }

private:
    static constexpr uint8_t DEV_ADDR = 0b01;

    // Register addresses
    static constexpr uint8_t REG_ADCDATA = 0x00;
    static constexpr uint8_t REG_CONFIG0 = 0x01;
    static constexpr uint8_t REG_CONFIG1 = 0x02;
    static constexpr uint8_t REG_CONFIG2 = 0x03;
    static constexpr uint8_t REG_CONFIG3 = 0x04;
    static constexpr uint8_t REG_IRQ     = 0x05;
    static constexpr uint8_t REG_MUX     = 0x06;

    // Fast commands
    static constexpr uint8_t FAST_START  = (DEV_ADDR << 6) | 0b101000;

    uint8_t _cs;
    uint8_t _irq;
    SPIClass& _spi;
    SPISettings _settings;
    float _vref;

    void writeReg(uint8_t addr, uint8_t data);
    uint8_t readReg(uint8_t addr);
    void fastCmd(uint8_t cmd);

    inline uint8_t cmdWrite(uint8_t addr) {
        return (DEV_ADDR << 6) | (addr << 2) | 0x02;
    }
    inline uint8_t cmdRead(uint8_t addr) {
        return (DEV_ADDR << 6) | (addr << 2) | 0x01;
    }
};
