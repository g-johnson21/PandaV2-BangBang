#pragma once
#include <Arduino.h>
#include <Wire.h>

// INA230 high-side/low-side current and power monitor
// I2C, 16-bit registers, big-endian

class INA230 {
public:
    INA230(TwoWire& bus, uint8_t addr);

    bool begin(float shuntOhms = 0.1f, float maxCurrent_A = 5.0f);

    float busVoltage_V();
    // Like busVoltage_V(), but reports an I2C failure instead of returning 0.
    bool readBusVoltage_V(float& out);
    float shuntVoltage_mV();
    float current_A();
    float power_W();

    // Alert: returns true if the alert pin has been asserted
    bool alertTriggered();
    void clearAlert();

    bool isConnected();

private:
    static constexpr uint8_t REG_CONFIG    = 0x00;
    static constexpr uint8_t REG_SHUNT_V   = 0x01;
    static constexpr uint8_t REG_BUS_V     = 0x02;
    static constexpr uint8_t REG_POWER     = 0x03;
    static constexpr uint8_t REG_CURRENT   = 0x04;
    static constexpr uint8_t REG_CAL       = 0x05;
    static constexpr uint8_t REG_MASK_EN   = 0x06;
    static constexpr uint8_t REG_ALERT_LIM = 0x07;

    // Bus voltage LSB = 1.25 mV
    static constexpr float BUS_V_LSB = 1.25e-3f;
    // Shunt voltage LSB = 2.5 µV
    static constexpr float SHUNT_V_LSB = 2.5e-6f;

    TwoWire& _wire;
    uint8_t _addr;
    float _currentLSB = 0;
    float _powerLSB   = 0;

    void writeReg16(uint8_t reg, uint16_t val);
    uint16_t readReg16(uint8_t reg);
    bool tryReadReg16(uint8_t reg, uint16_t& out);
};
