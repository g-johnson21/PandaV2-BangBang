# RTU V2 — Teensy 4.1 Pinout

Extracted from `Microcontroller.SchDoc` (dated 2026-03-31). Confidence levels noted where ambiguous — cross-check against ADC, RS-485, and Power schematic sheets before finalising `pins.h`.

---

## SPI Bus 0 — ADC1 (MCP3561RT, U23)

Default Teensy SPI (LPSPI4). ADC1 is the 24-bit delta-sigma ADC for the first 8-channel mux group.

| Teensy Pin | Net Name | Connected to |
|-----------|----------|--------------|
| 10 | CS_1 | MCP3561RT U23 /CS |
| 11 | MOSI | MCP3561RT U23 SDI (shared SPI0 bus) |
| 12 | MISO | MCP3561RT U23 SDO (shared SPI0 bus) |
| 13 | SCK | MCP3561RT U23 SCLK (shared SPI0 bus) |

**IRQ1** (MCP3561RT U23 /INT data-ready): **pin 6**. Active-low, open-drain.

---

## SPI Bus 1 — ADC2 + MCP23S17 (U38, U35)

Teensy SPI1 (LPSPI3). Shared bus between the second ADC and the I/O expander.

| Teensy Pin | Net Name | Connected to |
|-----------|----------|--------------|
| 26 / A12 | MOSI2 | MCP3561RT U38 SDI, MCP23S17 SI — *inferred from LPSPI3 pin availability; confirm against ADC2[8A]* |
| 27 / A13 | SCK2 | MCP3561RT U38 SCLK, MCP23S17 SCK — *inferred* |
| 39 / A15 | MISO2 | MCP3561RT U38 SDO, MCP23S17 SO — *inferred* |
| 37 | CS_3 | MCP23S17 U35 /CS — **confirmed on hardware** |
| 38 / A14 | CS_2 | MCP3561RT U38 /CS — **confirmed on hardware** |

**IRQ2** (MCP3561RT U38 /INT): **pin 14**. Active-low, open-drain.

**CS pins (confirmed on hardware):**
- **PIN_IOEXP_CS = 37** (MCP23S17 U35 chip select)
- **PIN_ADC2_CS = 38** (MCP3561RT U38 chip select)

> Both ADCs share their respective SPI bus with the downstream device (MCP23S17 on bus 1). CS lines are distinct. When the bus is idle, both CS must be deasserted high. The MCP23S17 driver wraps every transfer in `beginTransaction`/`endTransaction`, so actuator writes interleave safely with ADC2 conversions.

---

## I2C Bus 0 — Power Monitors (INA230, U8/U10/U12)

| Teensy Pin | Net Name | Connected to |
|-----------|----------|--------------|
| 18 / A4 | SDA | INA230 SDA (3× devices, Power[7A] sheet) |
| 19 / A5 | SCL | INA230 SCL (3× devices, Power[7A] sheet) |

Pull-ups: R44/R45 (4.7 Ω, 0.1%) from +3V3 — confirmed on schematic. **Note:** These are 4.7 Ω not 4.7 kΩ — double-check with the Power sheet; the PDF text reads "4.7 / 0.1% / 100mW" which matches 4.7 Ω power resistors. If these are actually 4.7 kΩ, R&D should confirm part numbers. At 3.3 V I2C must not exceed 400 kHz without verifying pull-up sizing.

INA230 I2C addresses (7-bit), set by A0/A1 pin strapping:

| Device | A1 | A0 | Address |
|--------|----|----|---------|
| U8  | GND | GND | 0x40 |
| U10 | GND | 3V3 | 0x41 |
| U12 | 3V3 | GND | 0x44 |

---

## UART — RS-485 Transceivers (ISL83491, U32_1/U32_2)

| Teensy Pin | Net Name | Connected to |
|-----------|----------|--------------|
| 0 / RX1 | RX1 | Serial1 RX → ISL83491 bus 1 RO |
| 1 / TX1 | TX1 | Serial1 TX → ISL83491 bus 1 DI |
| 7 / RX2 | RX2 | Serial2 RX → ISL83491 bus 2 RO |
| 8 / TX2 | TX2 | Serial2 TX → ISL83491 bus 2 DI |

**RS-485 DE (direction control) — hardware gap on built board:**

`/RE` is hardwired to GND (receiver always enabled). `DE` is driven by the `BREAK[1..3]` nets in the schematic, but those nets are **not routed to any Teensy GPIO** on the built board. The physical state of DE needs to be confirmed by inspecting the board.

Possible states and consequences:

| DE physical state | Effect |
|-------------------|--------|
| Tied to 3V3 (always high) | Driver always on. Firmware can transmit. Receiver simultaneously active so Teensy echoes its own TX. Other devices cannot win bus arbitration while this board is powered. |
| Tied to GND (always low) | Driver always off. Board is receive-only. TX calls do nothing on the bus. |
| Floating | ISL83491 has no internal pull on DE — undefined behavior, unreliable in both directions. |

**Recommended fix:** bodge wire from a free Teensy GPIO to the DE test point on each ISL83491. Free GPIO candidates on the built board: pins 9, 15, 16, 17, 24, 25, 29, 30, 31, 32. Assign `PIN_RS485_1_DE` and `PIN_RS485_2_DE` in `pins.h` once wired.

**Firmware approach until fixed:** receive-only operation is safe regardless of DE state (receiver always on via `/RE=GND`). Gate any transmit paths behind a `PIN_RS485_x_DE != -1` check so they compile but no-op until the hardware is resolved.

**RS-485 bus physical layer:** 120 Ω termination resistors (R79/R80) are on-board on both A and B lines. ESD protection (PESD2USB5UVT-QR) present on the RJ45 lines. Both ISL83491 VCC pins (13/14) are decoupled via R78 (10 kΩ) from +3V3. The firmware does not need to manage termination or ESD — these are fully passive.

**Note on additional UARTs:** Pins 28/29 (RX7/TX7) and 34/35 (RX8/TX8) are wired in the schematic. Purpose unclear from this sheet — may be for additional serial peripherals or debug. Verify on RS-485/IO sheets before use. Pins 30/31 (CRX3/CTX3) are CAN3 capable — also unassigned from this sheet.

---

## Analog Mux Channel Select — GPIOs

Three CD74HCT4067 16:1 muxes (U21, U24, U36) route analog inputs to the two ADCs.

### Mux A — feeds ADC1 (via U21)
4-bit address S_A0..S_A3 selects which of 16 channels is routed to ADC1.

| Teensy Pin | Net Name | Notes |
|-----------|----------|-------|
| **5** | S_A0 | Mux A select bit 0 |
| **4** | S_A1 | Mux A select bit 1 |
| **3** | S_A2 | Mux A select bit 2 |
| **2** | S_A3 | Mux A select bit 3 |

### Mux B — feeds ADC1 (via U24)
Second 16:1 mux for ADC1 channel bank B: load cells and thermocouples. **Not assembled on this board** — the firmware does not scan mux B at all.

| Teensy Pin | Net Name | Notes |
|-----------|----------|-------|
| **21 / A7** | S_B0 | Mux B select bit 0 |
| **20 / A6** | S_B1 | Mux B select bit 1 |
| **23 / A9** | S_B2 | Mux B select bit 2 |
| **22 / A8** | S_B3 | Mux B select bit 3 |

### Mux C — feeds ADC2 (via U36)
DC channel current sense: the INA181 output for `ACTUATE(n)` is on mux C channel **16−n** — the mux is wired in reverse of the actuator numbering (measured: ACTUATE5 → ch 11). Firmware maps this via `currentMuxCh()` in `BoardConfig.h`, so the `s…` telemetry row is in actuator order.

| Teensy Pin | Net Name | Notes |
|-----------|----------|-------|
| **35** | S_C0 | Mux C select bit 0 (CD74HCT4067 IC pin 10) |
| **36** | S_C1 | Mux C select bit 1 (CD74HCT4067 IC pin 11) |
| **33** | S_C2 | Mux C select bit 2 (CD74HCT4067 IC pin 14) |
| **34** | S_C3 | Mux C select bit 3 (CD74HCT4067 IC pin 13) |

> **EN wiring:** Mux C EN (IC pin 15) is hardwired to GND — always active, no firmware enable control. Same assumed for Mux A and B; confirm on ADC1 sheet. Because EN is always asserted, channel glitches during S-line transitions will reach the ADC input. Use the MCP3561RT's built-in OSR/DELAY to absorb settling (~115 ns mux + signal chain RC) before triggering a conversion after any channel change.

---

## MCP23S17 I/O Expander (U35) — ACTUATE[1..16]

SPI1 bus, CS = pin 37. Hardware address **A2=A1=A0 pulled HIGH → device address 7** (SPI opcode: write=`0x4E`, read=`0x4F`), per datasheet Figure 3-7 (`0100 A2 A1 A0 R/W`).

> Address matching only takes effect once IOCON.HAEN is set. Before that the device answers any address, so a mismatched opcode appears to work for exactly one transaction — the IOCON write that enables HAEN — and then the chip goes silent with IODIR still at its `0xFF` power-on default (all pins inputs). §3.3.2 also requires the address pins be externally biased *even when HAEN = 0*.

IOCON.HAEN must be set before address bits matter. Initialize all PORTB and PORTA pins as outputs, OLAT both registers to 0x00 before enabling outputs.

| MCP23S17 Pin | Register Bit | Board Net |
|-------------|--------------|-----------|
| GPB7 (28) | PORTB[7] | ACTUATE1 |
| GPB6 (27) | PORTB[6] | ACTUATE2 |
| GPB5 (26) | PORTB[5] | ACTUATE3 |
| GPB4 (25) | PORTB[4] | ACTUATE4 |
| GPB3 (24) | PORTB[3] | ACTUATE5 |
| GPB2 (23) | PORTB[2] | ACTUATE6 |
| GPB1 (22) | PORTB[1] | ACTUATE7 |
| GPB0 (21) | PORTB[0] | ACTUATE8 |
| GPA7 (20) | PORTA[7] | ACTUATE9 |
| GPA6 (19) | PORTA[6] | ACTUATE10 |
| GPA5 (18) | PORTA[5] | ACTUATE11 |
| GPA4 (17) | PORTA[4] | ACTUATE12 |
| GPA3 (16) | PORTA[3] | ACTUATE13 |
| GPA2 (15) | PORTA[2] | ACTUATE14 |
| GPA1 (14) | PORTA[1] | ACTUATE15 |
| GPA0 (13) | PORTA[0] | ACTUATE16 |

ACTUATE channels map to the 16-channel NPN transistor array (Q8_1..16, NSS30201MR6T1G) which drives downstream actuators/relays. A single PORTB write covers ACTUATE1–8; a single PORTA write covers ACTUATE9–16. During armed flight phases, OLAT writes must be guarded by a valid-state check before the SPI transaction is committed.

**INTA/INTB** (MCP23S17 interrupt outputs, pins 12/11): purpose unconfirmed on this sheet. Likely unconnected since the MCP23S17 is output-only here.

**RESET** (pin 9): ⚠️ **not connected on the built board.** This pin has no internal pull-up and the datasheet lists it as an External Reset Input, so a floating input can hold the device in reset. Needs a bodge to 3.3 V (direct or via 10 kΩ), or to a spare GPIO held high.

---

## Power

| Teensy Pin | Net | Notes |
|-----------|-----|-------|
| 105 | VIN | Main supply in, decoupled by C17 (10 µF 50 V) |
| 68 | VUSB | USB 5 V — can be used to detect USB connection |
| 43 | VBAT | RTC battery (B1, coin cell 502-3) |
| 101 | 3.3V_1 | Teensy 3.3 V output |
| 103 | 3.3V | Teensy 3.3 V output (alt pad) |
| 44 | 3.3V_2 | Teensy 3.3 V output (alt pad) |
| 100 | GND | — |

---

## Unassigned / Unconfirmed Pins

These appear in the schematic symbol but have no net connections visible on this sheet:

| Pin | Teensy Function | Status |
|-----|----------------|--------|
| 2 | OUT2 / GPIO | Likely IRQ1 — confirm on ADC1[8A] |
| 3–6 | GPIO | Likely S_A0..S_A3 — confirm on ADC1[8A] |
| 9 | OUT1C / GPIO | Possibly mux enable or spare |
| 20–22 (A6–A8) | Analog / GPIO | Possibly S_B0..S_B3 partial — confirm on ADC1[8A] |
| 24–25 (A10–A11) | Analog / GPIO | Unassigned or BREAK lines — confirm on RS-485 sheet |
| 28–29 | RX7/TX7 | Serial7 — purpose unclear |
| 30–31 | CRX3/CTX3 | CAN3 or Serial3 — purpose unclear |
| 32 | OUT1B / GPIO | Unassigned |
| 33–36 | GPIO | S_C2, S_C3, S_C0, S_C1 — confirmed (see Mux C) |
| 40–41 (A16–A17) | Analog / GPIO | Unassigned (not CS_2/CS_3 — those are 38/37) |

---

## What to pull from other sheets

| Sheet | Status |
|-------|--------|
| ADC1[8A] | ✅ S_A0..S_A3 (pins 5,4,3,2), S_B0..S_B3 (pins 21,20,23,22), IRQ1 (pin 6) — confirmed |
| ADC2[8A] | ✅ IRQ2 (pin 14), S_C0..S_C3 (pins 35,36,33,34), Mux C EN hardwired GND — confirmed |
| Power[7A] | ✅ INA230 addresses confirmed (U8=0x40, U10=0x41, U12=0x44); pull-up value still to verify |
| RS-485 / RTU V2[2C, 3C] | ✅ /RE hardwired GND, 120Ω termination confirmed; ❌ DE (BREAK[1..3]) not routed to Teensy on built board — hardware bodge required for TX |
