#pragma once
#include <Arduino.h>
#include "BoardConfig.h"

// Sensor channel mapping and conversion for PandaV2.
//
// Mux A (16ch) → ADC1 CH0: PTs through INA132 difference amps
// Mux B (16ch) → ADC1 CH1: solenoid current through INA181 (200x gain, 100mΩ shunt)
// Mux C (16ch) → ADC2 CH0: LCs (ch 0-7) through INA317/INA826; ch 8-15 unused
//
// Channel counts are configurable — adjust if your board populates fewer.

enum class SensorType : uint8_t {
    RAW,            // no conversion, report voltage
    PRESSURE,       // PT: voltage → PSI
    LOAD_CELL,      // LC: voltage → lbf (or N)
    CURRENT_SENSE   // solenoid current: voltage → amps
};

struct SensorCal {
    SensorType type;
    float scale;    // engineering_units = (voltage - offset) * scale
    float offset;   // voltage offset
};

// ── Mux C channel layout ───────────────────────────────────────────
// Channels 0..7  = load cells (INA317/INA826)
// Channels 8..15 = not scanned (no sensors set up)
// Adjust these if your board is wired differently.

static constexpr uint8_t MUX_C_LC_START  = 0;
static constexpr uint8_t MUX_C_LC_COUNT  = 8;

static constexpr uint8_t NUM_PT_CH = 16;
static constexpr uint8_t NUM_LC_CH = MUX_C_LC_COUNT;

// ── PT calibration ─────────────────────────────────────────────────
// Converts INA132 output voltage → loop current in milliamps.
// mA = (V_adc / PT_SHUNT_EFF_OHMS) * 1000
// All 4-20 mA sensors: 4 mA = 0 PSI, 20 mA = full-scale PSI.
// Per-channel PSI scaling is applied on the GC side using sensor_config.xlsx.

static constexpr SensorCal PT_DEFAULT = {SensorType::PRESSURE, 1.0f, 0.0f};

// ── Load cell calibration ──────────────────────────────────────────
// Default: report raw voltage. Replace with per-channel cal.

static constexpr SensorCal LC_DEFAULT = {SensorType::LOAD_CELL, 1.0f, 0.0f};

// ── Current sense conversion ───────────────────────────────────────
// INA181 at 200x gain with 100mΩ shunt:
// V_out = I_load * R_shunt * Gain = I_load * 0.1 * 200 = I_load * 20
// So I_load = V_adc / 20

static constexpr float CURRENT_SENSE_SCALE = 1.0f / 20.0f;  // A per V

// ── Conversion functions ───────────────────────────────────────────

inline float convertPT(float voltage, uint8_t ch) {
    (void)ch;
    // Return loop current in milliamps: mA = (V / R_shunt) * 1000
    return (voltage / PT_SHUNT_EFF_OHMS) * 1000.0f;
}

inline float convertLC(float voltage, uint8_t ch) {
    (void)ch;
    return (voltage - LC_DEFAULT.offset) * LC_DEFAULT.scale;
}

inline float convertCurrent(float voltage) {
    return voltage * CURRENT_SENSE_SCALE;
}
