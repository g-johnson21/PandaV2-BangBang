#include "INA230.h"

INA230::INA230(TwoWire& bus, uint8_t addr)
    : _wire(bus), _addr(addr)
{
}

bool INA230::begin(float shuntOhms, float maxCurrent_A) {
    if (!isConnected()) return false;

    // Config: 16 averages, 1.1ms conversion time for bus and shunt, continuous mode
    // bits [11:9] AVG=010 (16 avg), [8:6] VBUSCT=100 (1.1ms),
    // [5:3] VSHCT=100 (1.1ms), [2:0] MODE=111 (continuous shunt+bus)
    uint16_t config = 0x4527;
    writeReg16(REG_CONFIG, config);

    // Calculate calibration register
    // Current_LSB = maxCurrent / 2^15
    _currentLSB = maxCurrent_A / 32768.0f;
    _powerLSB   = _currentLSB * 25.0f;

    // CAL = 0.00512 / (Current_LSB * R_shunt)
    float calF = 0.00512f / (_currentLSB * shuntOhms);
    uint16_t cal = (uint16_t)calF;
    if (cal == 0) cal = 1;
    writeReg16(REG_CAL, cal);

    return true;
}

float INA230::busVoltage_V() {
    int16_t raw = (int16_t)readReg16(REG_BUS_V);
    return raw * BUS_V_LSB;
}

bool INA230::readBusVoltage_V(float& out) {
    uint16_t raw;
    if (!tryReadReg16(REG_BUS_V, raw)) return false;
    out = (int16_t)raw * BUS_V_LSB;
    return true;
}

float INA230::shuntVoltage_mV() {
    int16_t raw = (int16_t)readReg16(REG_SHUNT_V);
    return raw * SHUNT_V_LSB * 1000.0f;
}

float INA230::current_A() {
    int16_t raw = (int16_t)readReg16(REG_CURRENT);
    return raw * _currentLSB;
}

float INA230::power_W() {
    uint16_t raw = readReg16(REG_POWER);
    return raw * _powerLSB;
}

bool INA230::alertTriggered() {
    uint16_t mask = readReg16(REG_MASK_EN);
    return mask & 0x0010; // AFF (Alert Function Flag)
}

void INA230::clearAlert() {
    readReg16(REG_MASK_EN); // reading clears the flag
}

bool INA230::isConnected() {
    _wire.beginTransmission(_addr);
    return (_wire.endTransmission() == 0);
}

// --- private ---

void INA230::writeReg16(uint8_t reg, uint16_t val) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write((val >> 8) & 0xFF);
    _wire.write(val & 0xFF);
    _wire.endTransmission();
}

uint16_t INA230::readReg16(uint8_t reg) {
    uint16_t val = 0;
    tryReadReg16(reg, val);
    return val;
}

bool INA230::tryReadReg16(uint8_t reg, uint16_t& out) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;
    if (_wire.requestFrom(_addr, (uint8_t)2) != 2 || _wire.available() < 2)
        return false;
    uint8_t hi = _wire.read();
    uint8_t lo = _wire.read();
    out = (uint16_t(hi) << 8) | lo;
    return true;
}
