#include "MCP23S17.h"

MCP23S17::MCP23S17(uint8_t csPin, SPIClass& bus, SPISettings settings, uint8_t hwAddr)
    : _cs(csPin), _spi(bus), _settings(settings)
{
    _opWrite = OPCODE_BASE | ((hwAddr & 0x07) << 1);
    _opRead  = _opWrite | 0x01;
}

void MCP23S17::begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);

    // Enable hardware address pins (HAEN) so multiple devices can share bus
    writeReg(REG_IOCON, 0x08);

    // All pins as outputs (0 = output)
    writeReg(REG_IODIRA, 0x00);
    writeReg(REG_IODIRB, 0x00);

    // All outputs off at startup
    _state = 0;
    flushState();
}

bool MCP23S17::setChannel(uint8_t channel, bool state) {
    if (channel < 1 || channel > 16) return false;

    uint16_t bit = channelBit(channel);
    if (state)
        _state |= bit;
    else
        _state &= ~bit;

    // Only the port holding this channel changes, so write just that OLAT —
    // half the traffic on a bus shared with ADC2. The write still happens when
    // the bit was already set, so callers can re-assert a known state.
    if (channel <= 8)
        writeReg(REG_OLATB, (_state >> 8) & 0xFF);  // actuators 1-8
    else
        writeReg(REG_OLATA, _state & 0xFF);         // actuators 9-16
    return true;
}

void MCP23S17::setAll(uint16_t mask) {
    _state = mask;
    flushState();
}

void MCP23S17::allOff() {
    _state = 0;
    flushState();
}

uint16_t MCP23S17::readGpio() {
    uint8_t portB = readReg(REG_GPIOB);  // actuators 1-8
    uint8_t portA = readReg(REG_GPIOA);  // actuators 9-16
    return ((uint16_t)portB << 8) | portA;
}

bool MCP23S17::probe() {
    bool ok = true;
    for (uint8_t p : {0xA5, 0x5A}) {
        writeReg(REG_DEFVALA, p);
        if (readReg(REG_DEFVALA) != p) ok = false;
    }
    writeReg(REG_DEFVALA, 0x00);
    // begin() set both ports to outputs; 0xFF is the power-on/reset default.
    if (readReg(REG_IODIRA) != 0x00 || readReg(REG_IODIRB) != 0x00) ok = false;
    return ok;
}

bool MCP23S17::verifyState(uint16_t& readback) {
    uint8_t olatB = readReg(REG_OLATB);  // actuators 1-8
    uint8_t olatA = readReg(REG_OLATA);  // actuators 9-16
    readback = ((uint16_t)olatB << 8) | olatA;
    return readback == _state;
}

bool MCP23S17::selfTest(uint16_t& readback) {
    // 0xA55A: alternating on both ports, so a stuck-high or stuck-low bus,
    // a dead port, or a swapped port all produce a distinct mismatch.
    static constexpr uint16_t PATTERN = 0xA55A;

    setAll(PATTERN);
    uint8_t olatB = readReg(REG_OLATB);
    uint8_t olatA = readReg(REG_OLATA);
    readback = ((uint16_t)olatB << 8) | olatA;

    allOff();
    return readback == PATTERN;
}

// --- private ---

uint16_t MCP23S17::channelBit(uint8_t ch) {
    // Channels 1-8 → PORTB bits 7..0 (ch1=bit15 of _state, ch8=bit8)
    // Channels 9-16 → PORTA bits 7..0 (ch9=bit7 of _state, ch16=bit0)
    if (ch <= 8)
        return 1u << (16 - ch);  // PORTB: ch1→bit15, ch8→bit8
    else
        return 1u << (16 - ch);  // PORTA: ch9→bit7, ch16→bit0
}

void MCP23S17::flushState() {
    uint8_t portB = (_state >> 8) & 0xFF;  // actuators 1-8
    uint8_t portA = _state & 0xFF;          // actuators 9-16
    writeReg(REG_OLATB, portB);
    writeReg(REG_OLATA, portA);
}

void MCP23S17::writeReg(uint8_t addr, uint8_t data) {
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    _spi.transfer(_opWrite);
    _spi.transfer(addr);
    _spi.transfer(data);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

uint8_t MCP23S17::readReg(uint8_t addr) {
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    _spi.transfer(_opRead);
    _spi.transfer(addr);
    uint8_t val = _spi.transfer(0);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
    return val;
}
