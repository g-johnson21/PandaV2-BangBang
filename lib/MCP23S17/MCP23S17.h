#pragma once
#include <Arduino.h>
#include <SPI.h>

// MCP23S17 16-bit I/O expander driver
// All 16 pins configured as outputs driving actuator transistor array.
// Always write to OLAT registers (not GPIO) to avoid read-modify-write races.

class MCP23S17 {
public:
    MCP23S17(uint8_t csPin, SPIClass& bus, SPISettings settings, uint8_t hwAddr = 0);

    void begin();

    // Set a single actuator channel (1-indexed, 1..16). Returns false if out of range.
    bool setChannel(uint8_t channel, bool state);

    // Set all 16 channels at once (bit 0 = actuator 1, bit 15 = actuator 16)
    void setAll(uint16_t mask);

    // Turn all channels off. Call this on disarm.
    void allOff();

    // Read back the locally cached OLAT state (no bus traffic)
    uint16_t getState() const { return _state; }

    // --- Diagnostics ---
    // Read a register straight off the device. Use to confirm the SPI link is
    // live: if every register reads 0x00 or 0xFF, the chip isn't responding.
    uint8_t readRegister(uint8_t addr) { return readReg(addr); }
    void writeRegister(uint8_t addr, uint8_t data) { writeReg(addr, data); }

    // Read the actual pin states (GPIOA/GPIOB), not the cached OLAT value.
    // Layout matches getState(): bit15 = ACTUATE1 ... bit0 = ACTUATE16.
    uint16_t readGpio();

    // Write a test pattern to both OLATs and read it back. Returns true when
    // the readback matches, i.e. the SPI link and the device are both healthy.
    // Leaves all outputs OFF. Never call while anything is live on the bench.
    bool selfTest(uint16_t& readback);

    // Register addresses, exposed for diagnostic dumps
    static constexpr uint8_t REG_IODIRA  = 0x00;
    static constexpr uint8_t REG_IODIRB  = 0x01;
    static constexpr uint8_t REG_IOCON   = 0x0A;
    // DEFVALA has no effect on pin state unless interrupt-on-change is enabled
    // (it isn't here), so it doubles as a scratch register for link testing.
    // 0x06 is the BANK=0 address — 0x03 is DEFVALA only when IOCON.BANK=1,
    // and in BANK=0 that address is IPOLB (datasheet Table 3-1).
    static constexpr uint8_t REG_DEFVALA = 0x06;
    static constexpr uint8_t REG_GPIOA   = 0x12;
    static constexpr uint8_t REG_GPIOB   = 0x13;
    static constexpr uint8_t REG_OLATA   = 0x14;
    static constexpr uint8_t REG_OLATB   = 0x15;

private:
    static constexpr uint8_t OPCODE_BASE = 0x40;

    uint8_t _cs;
    SPIClass& _spi;
    SPISettings _settings;
    uint8_t _opWrite;
    uint8_t _opRead;
    uint16_t _state = 0;

    void writeReg(uint8_t addr, uint8_t data);
    uint8_t readReg(uint8_t addr);
    void flushState();

    // Map 1-indexed channel to port/bit
    // PORTB[7:0] → ACTUATE1..8  (GPB7=ACT1, GPB0=ACT8)
    // PORTA[7:0] → ACTUATE9..16 (GPA7=ACT9, GPA0=ACT16)
    static uint16_t channelBit(uint8_t ch);
};
