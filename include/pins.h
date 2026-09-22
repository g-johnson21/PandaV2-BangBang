#pragma once

// RTU V2 — Teensy 4.1 pin assignments
// Source: Microcontroller.SchDoc + ADC/RS-485/Power sheets (2026-03-31)
// See docs/pinout.md for full rationale, signal chain notes, and RS-485 DE status.

// ---------------------------------------------------------------------------
// SPI bus 0 — ADC1 (MCP3561RT, U23)
// Hardware SPI (LPSPI4): MOSI=11, MISO=12, SCK=13
// ---------------------------------------------------------------------------
#define PIN_ADC1_CS     10
#define PIN_ADC1_IRQ    6   // active-low, open-drain; attach interrupt INPUT_PULLUP
#define PIN_SPI0_MOSI   11
#define PIN_SPI0_MISO   12
#define PIN_SPI0_SCK    13

// ---------------------------------------------------------------------------
// SPI bus 1 — ADC2 (MCP3561RT, U38)
// Hardware SPI1 (LPSPI3): MOSI=26, MISO=39, SCK=27
// ---------------------------------------------------------------------------
#define PIN_SPI1_MOSI   26
#define PIN_SPI1_SCK    27
#define PIN_SPI1_MISO   39
#define PIN_ADC2_CS     38  // MCP3561RT U38
#define PIN_ADC2_IRQ    14  // active-low, open-drain; attach interrupt INPUT_PULLUP

// MCP23S17 I/O expander (U35) shares this bus — see docs/pinout.md.
// A2=A1=A0 pulled HIGH on the board → hardware address 7, so the SPI opcode is
// 0x4E (write) / 0x4F (read), per datasheet Figure 3-7: 0100 A2 A1 A0 R/W.
//
// This MUST match the strapping. begin() sets IOCON.HAEN, which enables address
// matching; get this wrong and the chip stops responding partway through init,
// leaving IODIR at its 0xFF power-on default (all pins inputs, nothing driven).
// Every transfer is wrapped in begin/endTransaction so it interleaves safely
// with ADC2 conversions on the same bus.
#define PIN_IOEXP_CS    37
#define IOEXP_HW_ADDR   0x07

// ---------------------------------------------------------------------------
// I2C bus 0 — power monitors (INA230 x3: U8, U10, U12)
// Hardware Wire (I2C0): SDA=18, SCL=19
// 4.7 Ω pull-ups to +3V3 on board (R44/R45)
// ---------------------------------------------------------------------------
#define PIN_I2C_SDA     18
#define PIN_I2C_SCL     19

// INA230 7-bit I2C addresses (A1/A0 pin strapping, confirmed from Power sheet)
#define INA230_ADDR_U8   0x40  // A1=GND, A0=GND
#define INA230_ADDR_U10  0x41  // A1=GND, A0=3V3
#define INA230_ADDR_U12  0x44  // A1=3V3, A0=GND

// ---------------------------------------------------------------------------
// UART — RS-485 transceivers (ISL83491, U32)
// /RE is hardwired to GND (receiver always enabled).
// DE is tied to +3V3 (driver always enabled).
// This makes the link full-duplex capable on each bus but always driving,
// so it's point-to-point only (no multi-drop bus sharing).
// DI connects to Teensy TX; RO connects to Teensy RX.
// ---------------------------------------------------------------------------
#define PIN_RS485_1_RX  28   // Serial7 RX
#define PIN_RS485_1_TX  29   // Serial7 TX
#define PIN_RS485_2_RX  25   // Serial6 RX
#define PIN_RS485_2_TX  24   // Serial6 TX

// DE is tied to +3V3 on both transceivers (driver always enabled).
// No GPIO control needed — the bus is always driven.
// This is safe for point-to-point links but precludes multi-drop RS-485.
#define PIN_RS485_1_DE  0xFF  // not GPIO-controlled
#define PIN_RS485_2_DE  0xFF  // not GPIO-controlled

// ---------------------------------------------------------------------------
// Analog mux channel select — CD74HCT4067 x3 (U21, U24, U36)
// EN is hardwired to GND on all three (always active).
// Write S0..S3, wait for mux + signal chain settling, then trigger ADC.
// Use MCP3561RT DELAY register — do not poll immediately after channel switch.
// ---------------------------------------------------------------------------

// Mux A (U21) — feeds ADC1 (voltage / differential channels)
#define PIN_MUX_A_S0    5
#define PIN_MUX_A_S1    4
#define PIN_MUX_A_S2    3
#define PIN_MUX_A_S3    2

// Mux B (U24) — feeds ADC1 (current sense channels)
#define PIN_MUX_B_S0    21
#define PIN_MUX_B_S1    20
#define PIN_MUX_B_S2    23
#define PIN_MUX_B_S3    22

// Mux C (U36) — feeds ADC2
// Pin 36 is the Teensy hardware CS_2 function but is used here as plain GPIO.
#define PIN_MUX_C_S0    35
#define PIN_MUX_C_S1    36
#define PIN_MUX_C_S2    33
#define PIN_MUX_C_S3    34

// ---------------------------------------------------------------------------
// Arming
// PIN_ARM is a level held HIGH while armed (enables the solenoid drive) and
// LOW while disarmed — see ArmingController and test/dc_channel_test.
// PIN_DISARM is redundant on V2 and is deliberately not driven by firmware.
// ---------------------------------------------------------------------------
#define PIN_ARM     41
#define PIN_DISARM  16  // Redundant — not driven

// ---------------------------------------------------------------------------
// Power / housekeeping
// ---------------------------------------------------------------------------
#define PIN_VBAT  43  // RTC coin cell backup (B1, 502-3)
