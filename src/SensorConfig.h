#pragma once
#include <Arduino.h>
#include "BoardConfig.h"

// Sensor channel mapping and conversion for PandaV2.
//
// Mux A (16ch) → ADC1 CH0: PTs through INA132 difference amps
// Mux B (16ch) → ADC1 CH1: load cells + thermocouples — NOT ASSEMBLED on this
//                          board, so mux B is not scanned at all. Restore the
//                          bank here and in main.cpp if those parts go on.
// Mux C (16ch) → ADC2 CH0: DC channel current sense through INA181A1
//                          (x20 gain, 100mΩ shunt). ACTUATE(n) ↔ mux C ch
//                          16-n (reversed; see currentMuxCh()).

enum class SensorType : uint8_t {
    RAW,            // no conversion, report voltage
    PRESSURE,       // PT: voltage → PSI
    CURRENT_SENSE   // solenoid current: voltage → amps
};

struct SensorCal {
    SensorType type;
    float scale;    // engineering_units = (voltage - offset) * scale
    float offset;   // voltage offset
};

static constexpr uint8_t NUM_PT_CH = 16;

// DC current-sense channels, 1:1 with the expander outputs: curData index i is
// the current through ACTUATE(i+1), read from mux C ch currentMuxCh(i+1).
static constexpr uint8_t NUM_CURRENT_CH = NUM_MUX_C_CH;
static_assert(NUM_CURRENT_CH == NUM_ACTUATORS,
              "current sense is 1:1 with the DC actuator channels");

// ── PT calibration ─────────────────────────────────────────────────
// Converts INA132 output voltage → loop current in milliamps.
// mA = (V_adc / PT_SHUNT_EFF_OHMS) * 1000
// All 4-20 mA sensors: 4 mA = 0 PSI, 20 mA = full-scale PSI.
// Per-channel PSI scaling is applied on the GC side using sensor_config.xlsx.

static constexpr SensorCal PT_DEFAULT = {SensorType::PRESSURE, 1.0f, 0.0f};

// ── Current sense conversion ───────────────────────────────────────
// INA181A1 (fixed x20 gain) with a 100mΩ shunt (R43):
//   V_out = I_load * R_shunt * Gain = I_load * 0.1 * 20 = I_load * 2
//   I_load = V_adc / 2
//
// This matches V1's carried-over `sConstant = 0.5`, which was the same
// amplifier chain expressed as A-per-volt.
//
// RANGE: ADC2's reference is ADC2_VREF_V (3.27 V), so the chain saturates at
// 3.27 / 2 ≈ 1.64 A. Readings at that value are clipped, not real.

static constexpr float CURRENT_SENSE_SCALE = 0.5f;  // A per V (1 / (20 * 0.1Ω))
static constexpr float CURRENT_FULL_SCALE_A = ADC2_VREF_V * CURRENT_SENSE_SCALE;

// ── Conversion functions ───────────────────────────────────────────

inline float convertPT(float voltage, uint8_t ch) {
    (void)ch;
    // Return loop current in milliamps: mA = (V / R_shunt) * 1000
    return (voltage / PT_SHUNT_EFF_OHMS) * 1000.0f;
}

inline float convertCurrent(float voltage) {
    return voltage * CURRENT_SENSE_SCALE;
}
