# RTU V2 Datasheet Manifest

Firmware-relevant components only. Passives (R, C, L), LEDs, connectors, and simple protection diodes are omitted — they have no firmware interface.

---

## Microcontroller

### Teensy 4.1 — `IC1`
- **MCU core:** NXP iMXRT1062 (600 MHz Cortex-M7)
- **Datasheet/docs:** https://www.pjrc.com/store/teensy41.html
- **NXP iMXRT1060 Reference Manual:** https://www.nxp.com/docs/en/reference-manual/IMXRT1060RM.pdf (3700+ pages — download separately, do not commit)
- **Pinout card:** https://www.pjrc.com/teensy/card11a_rev4_web.pdf
- **Key interfaces in use on this board:** SPI (ADC + I/O expander), I2C (power monitors), UART (RS-485), GPIO
- **Notes:** Teensy 4.1 runs at 3.3 V logic. USB is available for debug. No onboard EEPROM — use EEPROM emulation in flash or the external flash chip on the Teensy. The iMXRT1062 has 7 UARTs; RS-485 direction control (DE/RE) must be toggled in firmware around each transmit frame.

---

## ADC

### MCP3561RT-E/ST — `U23, U38`
- **File:** `datasheets/MCP3561RT.pdf`
- **Interface:** SPI (mode 0,0 or 1,1 — see §6.1), up to 20 MHz
- **Resolution:** 24-bit delta-sigma, single channel, internal 2.4 V reference
- **Key registers:** CONFIG0–CONFIG3, IRQ, MUX, SCAN, TIMER, OFFSETCAL, GAINCAL, LOCK, CRCCFG, ADCDATA
- **Firmware notes:**
  - Continuous conversion or one-shot; read via STATUS+DATA burst (§7.3)
  - CRCCFG enables CRC on SPI frames — worth enabling for flight hardware reliability
  - Data-ready signaled on /INT pin; poll or attach interrupt
  - Gain (1×–64×) and OSR (32–98304) are software-configurable; set once at init
  - Two instances on board — assign separate CS lines

---

## I/O Expander

### MCP23S17T-E/SS — `U35`
- **File:** `datasheets/MCP23S17.pdf`
- **Interface:** SPI (mode 0,0 or 1,1), up to 10 MHz
- **I/O:** 16-bit bidirectional GPIO split into PORTA and PORTB
- **Key registers:** IODIRA/B, IPOLA/B, GPINTENA/B, DEFVALA/B, INTCONA/B, IOCON, GPPUA/B, INTFA/B, INTCAPA/B, GPIOA/B, OLATA/B
- **Firmware notes:**
  - IOCON.HAEN must be set to enable hardware address pins (A0–A2); only one device here so address = 0b000
  - IOCON.MIRROR can tie INTA/INTB together — useful if only one interrupt line is wired
  - Use OLAT not GPIO for writes to avoid read-modify-write race on output ports
  - Initialize all pins as inputs with pull-ups disabled; explicitly configure direction at startup

---

## Analog Multiplexer

### CD74HCT4067PWR — `U21, U24, U36`
- **File:** `datasheets/CD74HCT4067.pdf`
- **Interface:** 4-bit GPIO control (S0–S3) + EN; no SPI
- **Type:** 16:1 single-channel analog mux, TTL-compatible control inputs, 5 V supply
- **Firmware notes:**
  - Select channel by writing 4-bit address to S0–S3 before triggering ADC conversion
  - EN (active-low) must be asserted; deassert when idle to tristate the common line
  - Switch settling time ~115 ns typical — add a small delay (or use ADC's built-in OSR settling) after channel change before sampling
  - Three instances: likely routing analog front-end channels to the two ADCs

---

## Signal Conditioning (analog front-end — no firmware register interface)

### INA132UA/2K5 — `U19_1..16` (16×)
- **File:** `datasheets/INA132.pdf`
- **Type:** Fixed-gain (×1 to ×10) difference amplifier, 18 V supply
- **Firmware relevance:** Sets the gain in the analog chain for differential voltage inputs. Gain is set by external resistors — firmware reads the scaled output via ADC. Know the gain to convert ADC codes to engineering units.

### INA181A1IDBVR — `U42_1..16` (16×)
- **File:** `datasheets/INA181.pdf`
- **Type:** Current sense amplifier, fixed ×20 gain (A1 variant), SOT-23-6
- **Firmware relevance:** 16-channel DC-channel current sense, one per `ACTUATE1..16`, routed through mux C to ADC2 in reverse order (`ACTUATE(n)` → mux C ch 16−n). At ×20 gain with the 100 mΩ shunt resistors (R43): `I = V_out / (20 × R_shunt) = V_out / 2`. ADC2's 3.27 V reference puts full scale at ≈1.64 A per channel.

### INA317IDGKR — `U18_1..8` (8×)
- **File:** `datasheets/INA317.pdf`
- **Type:** Precision instrumentation amp, gain set by single external resistor, 1.8–5.5 V
- **Firmware relevance:** Gain = 1 + (200 kΩ / R_G). Know R_G to convert to engineering units.

### INA826AIDRGR — `U20_1..8` (8×)
- **File:** `datasheets/INA826.pdf`
- **Type:** Precision instrumentation amp, 36 V supply, 200 µA supply current
- **Firmware relevance:** Same as INA317. Gain set by R_G. Used on higher-voltage input channels.

### OPA210IDBVR — `U22, U25, U28, U31, U37`
- **File:** `datasheets/OPA210.pdf`
- **Type:** 36 V, 2.2 nV/√Hz low-power op-amp, SOT-23-5
- **Firmware relevance:** Buffer/filter stage in signal chain. No register interface — relevant for understanding noise floor and bandwidth of the ADC input path.

---

## Power Monitor

### INA230AIDGST — `U8, U10, U12`
- **File:** `datasheets/INA230.pdf`
- **Interface:** I2C, up to 2.94 MHz (Fast-mode+), address set by A0/A1 pins
- **Key registers:** Configuration (0x00), Shunt voltage (0x01), Bus voltage (0x02), Power (0x03), Current (0x04), Calibration (0x05), Mask/Enable (0x06), Alert limit (0x07)
- **Firmware notes:**
  - Write CALIBRATION register: `CAL = 0.00512 / (Current_LSB × R_shunt)` — sets scale for current and power registers
  - Alert pin (active-low, open-drain) can trigger on over-current, over-voltage, etc. — wire to MCU interrupt for fault detection
  - Three instances; each has a distinct I2C address via A0/A1 pin strapping — read board schematic to confirm addresses
  - Averaging register (AVG[2:0]) and conversion time (VBUSCT, VSHCT) trade latency for noise — tune at init

---

## RS-485 Transceiver

### ISL83491IBZ-T — `U32_1, U32_2`
- **File:** `datasheets/ISL83491.pdf`
- **Interface:** UART (RX/TX from MCU) + 2 GPIO direction control pins (DE, /RE)
- **Specs:** 3.6 V supply, ±15 kV ESD on bus pins, 10 Mbps max, -7 V to +12 V common-mode range
- **Firmware notes:**
  - DE (driver enable, active-high) and /RE (receiver enable, active-low) are separate — both must be driven
  - For half-duplex: assert DE, transmit complete frame, deassert DE before first byte can echo back. Timing is critical: deassert within one bit-time of stop bit. Use hardware flow control or DMA-complete interrupt if available
  - Two transceivers suggest two independent RS-485 buses (or one differential pair each for balanced redundancy)
  - Over hundreds of feet: use 120 Ω termination at each end; limit baud to ≤115200 without repeaters unless cabling is well-characterized
  - /RE pulled low = always receiving; useful for listen-before-talk or collision detection

---

## Voltage References

### REF35120QDBVR — `U30`
- **File:** `datasheets/REF35.pdf`
- **Output:** 1.2 V, ±0.05%, 12 ppm/°C, 650 nA quiescent, AEC-Q100 qualified
- **Firmware relevance:** Likely used as a precision reference for one or both ADCs (or as a known calibration voltage). No register interface. Firmware should not rely on ADC self-calibration alone — cross-check against this reference at startup if routed to an ADC input.

---

## Power Management

### LMR54410DBVR — `U1, U5, U13`
- **File:** `datasheets/LMR54410.pdf`
- **Type:** 3.2–65 V input, 1 A synchronous buck, 1.1 MHz
- **Firmware relevance:** Power supply — no firmware interface. Relevant for startup sequencing: board supply rails may need to be up before enabling downstream loads. Monitor via INA230 if rail health is wired there.

### LM74502DDFR — `U3, U4, U7, U9, U11, U14, U15`
- **File:** `datasheets/LM74502.pdf`
- **Type:** 3.2–65 V RPP (reverse polarity protection) controller with OVP and load disconnect
- **Firmware relevance:** Hardware protection — no register interface. The load disconnect output can cut power to downstream sections on OVP/RPP events. Know which rails each unit protects.

### MCP1826ST-3302E/DB — `U6`
- **File:** `datasheets/MCP1826.pdf`
- **Type:** 1 A LDO, 3.3 V output
- **Firmware relevance:** 3.3 V rail for MCU and logic. No firmware interface.

### MCP1826ST-5002E/DB — `U2`
- **File:** `datasheets/MCP1826.pdf`
- **Type:** 1 A LDO, 5.0 V output
- **Firmware relevance:** 5 V rail, likely for analog mux supply and HCT logic family. No firmware interface.

---

## Discrete Logic

### SN74LVC1G04DBVR — `U26, U29, U33_1, U33_2, U34_1, U34_2, U39, U40`
- **File:** `datasheets/SN74LVC1G04.pdf`
- **Type:** Single inverter, 1.65–5.5 V, SOT-23-5

### SN74LVC1G17DBVR — `U48_1, U48_2, U48_3`
- **File:** `datasheets/SN74LVC1G17.pdf`
- **Type:** Single Schmitt-trigger buffer, 1.65–5.5 V

### SN74LVC1G373DCKR — `U43`
- **File:** `datasheets/SN74LVC1G373.pdf`
- **Type:** Single D-type transparent latch with 3-state output, 1.65–5.5 V

### SN74LVC1404DCUR — `U27`
- **File:** `datasheets/SN74LVC1404.pdf`
- **Type:** 2-channel Schmitt-trigger inverter, 1.65–5.5 V, VSSOP-8
- **Firmware notes (logic family):** LVC is 3.3 V native but 5 V tolerant on inputs. HCT (CD74HCT4067) requires TTL-level inputs — LVC outputs (V_OH ≥ 2.4 V) are sufficient to drive HCT inputs (V_IH min 2.0 V) at 3.3 V supply.

---

## MOSFETs (switched outputs)

### CSD18510Q5B — `Q1, Q4, Q5, Q6`
- **File:** `datasheets/CSD18510Q5B.pdf`
- **Type:** 40 V N-channel NexFET, 0.96 mΩ R_DS(on), SON-8
- **Firmware notes:** Gate driven from MCU GPIO (likely through gate resistor). Fully on at V_GS = 3.3 V (check gate threshold — typical V_GS(th) ~1.2 V min, logic-level compatible). Likely used for high-current load switching.

### CSD17573Q5B — `Q2, Q3, Q7`
- **File:** `datasheets/CSD17573Q5B.pdf`
- **Type:** 30 V N-channel NexFET, 1.45 mΩ R_DS(on), SON-8
- **Firmware notes:** Same drive considerations as CSD18510. Lower V_DS rating suggests lower-voltage switched rails.

---

## Crystal

### Epson TSX-3225 16 MHz — `Y1`
- **Datasheet:** https://www5.epsondevice.com/en/products/crystal/tsx3225.html
- **Specs:** 16 MHz, ±10 ppm, 9 pF load, 60 Ω ESR, SMD 3.2×2.5 mm
- **Firmware notes:** External crystal for Teensy 4.1. The iMXRT1062 uses this as the RTC crystal input or for the main PLL — confirm in board schematic. Teensy's bootloader configures the PLL; main firmware clock config should match.

---

## Relay

### TE Connectivity 1-1415899-3 (RZ03 series) — `R102`
- **Type:** SPST-NO, 5 VDC coil, 16 A contacts
- **Firmware notes:** Driven by NPN transistor (Q8, NSS30201MR6T1G) via GPIO. GPIO high → transistor on → relay energizes. Ensure flyback diode (D12 array, PMEG4010EJ) is present across coil — it is on this board. Relay actuation (~7 ms operate, ~3 ms release) should be treated as a slow asynchronous event; do not assume state change is instantaneous.

---

## Not downloaded (fetch manually)

| Resource | URL |
|---|---|
| NXP iMXRT1060 Reference Manual | https://www.nxp.com/docs/en/reference-manual/IMXRT1060RM.pdf |
| Teensy 4.1 schematic | https://www.pjrc.com/teensy/schematic41.png |
| Teensy 4.1 pinout card | https://www.pjrc.com/teensy/card11a_rev4_web.pdf |

The iMXRT1060 RM is ~3700 pages; keep it local and outside the repo.
