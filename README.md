# PandaV2 Firmware

Firmware for the Remote Terminal Unit V2 DAQ board.

**MCU:** Teensy 4.1 (NXP iMXRT1062, 600 MHz Cortex-M7)
**Comms:** RS-485 (ISL83491, x2) — designed to operate reliably over hundreds of feet
**ADC:** MCP3561RT 24-bit delta-sigma (x2, SPI)
**I/O expander:** MCP23S17 16-bit (SPI)
**Power monitor:** INA230 (I2C, x3)
**Analog front-end:** 16-ch difference amp (INA132), 16-ch current sense (INA181), 8-ch instrumentation amp (INA317/INA826), 16:1 mux (CD74HCT4067)

## Structure

```
src/          application source
include/      shared headers
lib/          driver libraries (one dir per IC)
docs/
  datasheets/ IC datasheets
  manifest.md datasheet index and firmware notes per part
  GC_USERS_GUIDE.md      GC command + telemetry protocol (canonical)
  BANGBANG_FLOWCHART.md  bang-bang state machine and per-tick logic
```

## Safety notes

- Board may be armed in-flight; all state transitions must be explicit and guarded
- RS-485 lines are long-haul; assume noise, framing errors, and partial packets
- All ADC reads should validate range and flag faults rather than silently clamp
