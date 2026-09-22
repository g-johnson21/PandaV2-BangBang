#pragma once
#include <SPI.h>

// SPI bus 0 — ADC1 is alone on this bus, so it runs at the MCP3561RT's rate.
static const SPISettings SPI_ADC_SETTINGS(20000000, MSBFIRST, SPI_MODE0);

// SPI bus 1 — ONE setting for every device on the bus (ADC2 + MCP23S17), so
// clock, mode and bit order never change between transactions no matter which
// chip is addressed.
//
// The rate is set by the slowest device on the bus: the MCP23S17 is rated
// 10 MHz max (datasheet p.1, "High-Speed SPI Interface: 10 MHz (maximum)").
// Do not raise past 10 MHz — that puts the expander out of spec. It currently
// runs at 1 MHz (a conservative bring-up value): an expander write takes
// ~26 µs and an ADC2 sample read ~34 µs. 10 MHz would cut those ~10x.
//
// Both drivers still wrap every transfer in begin/endTransaction — that is what
// makes sharing safe. This just removes the reconfiguration between devices.
static const SPISettings SPI_BUS1_SETTINGS(1000000, MSBFIRST, SPI_MODE0);

// ADC reference voltages (V) — full-scale for count → volts conversion:
//   V = VREF * raw / 2^23
// These are SCALE FACTORS only. They do not select the reference; the chip's
// reference source is set by CONFIG0.VREF_SEL in MCP3561RT::begin().
//
// TODO: verify against the schematic. docs/manifest.md lists U30 as a
// REF35120, which is 1.2 V (REF35 datasheet p.3) — 1.25 V is the REF35125.
// If U30 drives REFIN+ on an ADC, that ADC's readings are ~4.2% high at 1.25.
static constexpr float ADC1_VREF_V = 1.193f; // U23
static constexpr float ADC2_VREF_V = 3.27f; // U38

// ADC timing
static constexpr uint32_t T_MUX_SETTLE_US = 500;

// Serial
static constexpr uint32_t RS485_BAUD = 460800;
static constexpr uint32_t DEBUG_BAUD = 115200;
static constexpr size_t RS485_RX_BUF = 512;
static constexpr size_t RS485_TX_BUF = 2048;
static constexpr uint32_t PACKET_IDLE_MS = 100;

// Channel counts

// Solenoid drive is routed through the MCP23S17 I/O expander (U35), not Teensy
// GPIO. Channel n (1-indexed) → ACTUATEn → transistor array Q8_n.
// The expander provides 16 channels; the wire protocol addresses a channel with
// a single hex digit, so only 1..15 are reachable over RS-485.
static constexpr uint8_t NUM_EXPANDER_CH = 16;

// Actuators exposed to the control software. Raise toward 15 as the downstream
// channels are populated — every loop and range check keys off this constant.
static constexpr uint8_t NUM_ACTUATORS = 16;
static constexpr uint8_t NUM_MUX_A_CH = 16; // voltage/differential via mux A
// Mux B carries load cells + thermocouples, which are NOT assembled on this
// board — it is not scanned. Its pins stay defined in pins.h.
static constexpr uint8_t NUM_MUX_C_CH = 16; // DC current sense via mux C (ADC2)

// Mux C is wired in reverse of the actuator numbering: ACTUATE(n) (1..16) is
// sensed on mux C channel 16-n (measured: ACTUATE5 -> mux C ch 11).
static constexpr uint8_t currentMuxCh(uint8_t actuator) {
    return NUM_MUX_C_CH - actuator;
}
static constexpr uint8_t NUM_MAX_COMMANDS = 64;

// ADC conversion watchdog. A healthy MCP3561RT conversion finishes in well
// under 2 ms; anything past this is a hung ADC. The scanner abandons that
// sample and moves on rather than stalling every channel behind it.
static constexpr uint32_t ADC_CONV_TIMEOUT_US = 10000;

// Telemetry identifiers (must match control software)
static constexpr char ID_PT = 'p';           // PT loop current (mA), all mux A channels
static constexpr char ID_PT_PSI = 'P';       // BB PT pressure (PSI): scaled, tared, median-filtered
static constexpr char ID_SOLENOID_CURRENT = 's';
static constexpr char ID_POWER = 'v';
static constexpr char ID_GC_HEARTBEAT = 'h';

// Conversion constants (carry forward from V1, recalibrate on V2 hardware)
static constexpr uint8_t DATA_DECIMALS = 5;

// PT current-sense shunt effective resistance (Ω).
// INA132 at unity gain with shunt in the 4-20 mA loop.
// Empirically derived: V_adc ≈ 0.023 V at 4 mA idle → R = 0.023/0.004 = 5.75 Ω.
// Verify against schematic; standard value is likely 5.6 Ω (E24).
static constexpr float PT_SHUNT_EFF_OHMS = 47.0f;

// PT loop current → PSI for the bang-bang PTs (4-20 mA transducers).
static constexpr float PT_ZERO_MA = 4.0f;
static constexpr float PT_SPAN_MA = 16.0f;                      // 20 - 4
static constexpr float PT_FULL_SCALE_PSI = 1500.0f * 0.9748f;   // Sensor rating times scale

// ── Telemetry cadence ───────────────────────────────────────────────
// Keep the command bus idle most of the time. Periodic telemetry must never
// continuously refill the UART TX buffer or GC cannot get a command (most
// importantly a disable/disarm) onto the wire. Periodic frames are skipped
// whenever the TX buffer lacks TX_PRIORITY_RESERVE bytes of headroom, which
// stays free for command responses and EVT: lines.
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 50;       // 20 Hz
static constexpr uint32_t BB_HEARTBEAT_INTERVAL_MS = 1000;  // 1 Hz
static constexpr uint32_t BB_DEBUG_INTERVAL_MS = 100;       // 10 Hz
static constexpr uint32_t POWER_INTERVAL_MS = 500;          // 2 Hz
// I/O expander liveness on the USB debug console (not RS-485).
static constexpr uint32_t IOEXP_DEBUG_INTERVAL_MS = 1000;   // 1 Hz
static constexpr size_t TX_PRIORITY_RESERVE = 256;

// ── GC link watchdog ────────────────────────────────────────────────
// GC only talks to the board when the operator acts, so silence by itself is
// NOT evidence of a dead link. GC must send a periodic heartbeat ('h') for the
// board to tell "quiet" apart from "gone".
//
// The watchdog is HEARTBEAT-GATED: dormant until the first 'h' of the boot,
// then latched armed until reset. A GC that never sends heartbeats gets no
// protection but no nuisance disarms either. Whether the watchdog is armed is
// published every second on the LINK: telemetry line.
//
// GC should send 'h' at 5 Hz. Three missed beats trip stage 1.
static constexpr uint32_t COMMS_LOSS_MS = 600;      // stage 1: bang-bang safe
static constexpr uint32_t COMMS_DISARM_MS = 10000;  // stage 2: full disarm
static_assert(COMMS_LOSS_MS < COMMS_DISARM_MS,
              "BB must be forced safe before the disarm stage runs");

// =================== Bang-Bang configuration =================== //

// Sentinel value for an unset DC channel (DC channels are 1-indexed).
static constexpr uint8_t BB_DC_CH_UNSET = 0;

// PT channels (0-indexed into mux A / ptData[]). These are sampled at high
// priority by the ADC1 scanner and converted into the PSI array BB reads.
static constexpr uint8_t BB_LOX_PT_CH = 0;  // LOX pressure sensor
static constexpr uint8_t BB_FUEL_PT_CH = 1; // Fuel pressure sensor

// Number of PT channels carried in the PSI view (ptPsiData[] / 'P' row).
// Index i of the PSI view is mux A channel i.
static constexpr uint8_t NUM_PSI_PT_CH = 2;
static_assert(BB_LOX_PT_CH < NUM_PSI_PT_CH && BB_FUEL_PT_CH < NUM_PSI_PT_CH,
              "BB PT channels must lie inside the PSI view");

// DC press-solenoid channels (1-indexed, maps to expander ACTUATEn).
// TODO(user): confirm against the V2 harness before a hot-fire.
static constexpr uint8_t BB_LOX_DC_CH = 1;
static constexpr uint8_t BB_FUEL_DC_CH = 2;

// DC vent-solenoid channels (1-indexed). Set to BB_DC_CH_UNSET to disable
// auto-vent / abort for that side; those commands are then rejected with
// EVT:...:AV_NO_HW.
static constexpr uint8_t BB_LOX_VENT_DC_CH = 4;  // LOX vent solenoid
static constexpr uint8_t BB_FUEL_VENT_DC_CH = 7; // Fuel vent solenoid

// Sanity bounds — BB latches ABORT if the PT reading falls outside this range
// (ADC fault or open sensor).
static constexpr float BB_PRESSURE_MIN_PSI = -50.0f;
static constexpr float BB_PRESSURE_MAX_PSI = 4000.0f;

// ~50 ms without a complete BB PT sweep force-safes any active BB controller;
// the operator must explicitly re-enable after data resumes.
static constexpr uint32_t BB_PT_STALE_MS = 50;

// BB consumes a rolling-median PSI signal. A monotonic ramp is delayed by half
// this window, so predictive cutoff compensates that measured delay in
// addition to the configured mechanical valve-close delay.
static constexpr uint8_t PT_PSI_MEDIAN_WINDOW = 75;

// Predictive press-valve cutoff. Pressure rate is low-pass filtered so one
// noisy PT derivative does not command an early close. The mechanical delay
// itself is persisted per side in BBConfig and defaults to 15 ms.
static constexpr float BB_PRESSURE_RATE_ALPHA = 0.20f;
static constexpr uint32_t BB_RATE_RESET_MS = 100;

// EEPROM layout for BB config persistence. Magic is bumped whenever the
// BBConfig struct layout changes; old EEPROM contents are ignored on mismatch.
static constexpr uint16_t BB_EEPROM_MAGIC = 0xBB46;
static constexpr int BB_EEPROM_ADDR = 0;

// PT tare offsets (PSI subtracted after mA→PSI conversion). Separate EEPROM
// block so layout changes do not collide with BB config.
static constexpr uint16_t PT_TARE_EEPROM_MAGIC = 0x5441; // "TA"
static constexpr int PT_TARE_EEPROM_ADDR = 256;
