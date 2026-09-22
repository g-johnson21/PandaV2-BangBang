# PandaV2 Ground Control (GC) Users Guide

This is the canonical reference for every line sent to and received from PandaV2 over RS-485. The command set, bang-bang behavior, and event stream match Panda V1 (`panda-firmware-main`); differences come from V2's hardware and are called out below. If something behaves differently from what is described here, treat it as a bug in the firmware, not a docs gap.

## 1. Transport

| Link | Direction | Baud | Framing | Purpose |
|---|---|---|---|---|
| RS-485 bus 1 (`Serial7`, pins 28/29) | GC ↔ V2 | 460800 | newline-terminated ASCII | commands + telemetry |
| RS-485 bus 2 (`Serial6`, pins 25/24) | GC ↔ V2 | 460800 | newline-terminated ASCII | commands + telemetry (identical to bus 1) |
| USB CDC (`Serial`) | PC ↔ V2 | 115200 | debug | developer console only |

Both RS-485 buses are equivalent GC links. Commands are accepted on either; direct replies (`Arming!`, `BB_ERROR:…`, `SEQ_ACK:…`, …) go back on the bus the command arrived on. Periodic telemetry, `EVT:` lines, and `SEQ_STEP`/`SEQ_EXEC_COMPLETE` go out on both.

Every packet is a single line terminated by `\n` (or `\r`). Packets time out after 100 ms idle (`PACKET_IDLE_MS`) even if no newline is seen, so a dropped delimiter is recoverable.

Unlike V1, V2 reads its own PTs — there is no V2→V1 crossover. The bang-bang PTs (mux A channels 0 and 1) are sampled on ADC1 between every other channel, so they refresh at a few hundred Hz; the rest of mux A and all of mux B share the remaining ADC1 time.

## 2. Command reference

`<side>` is `L` (LOX) or `F` (Fuel). All numeric fields are ASCII decimal.

### 2.1 Arming

| Command | Effect |
|---|---|
| `a` | Drive `PIN_ARM` (41) **high and hold it**. This enables the solenoid drive stage. Required before BB `b…1`, `e…1`, `v…1`. |
| `r` | Drive `PIN_ARM` low. **Also**: `forceSafe()` on both BB controllers (press + vent closed, ABORT latch cleared, state → DISABLED), cancel any running sequence, and turn every expander output off. |

Response lines: `Arming!`, `Disarming!`, `SEQ_ABORT: Outputs de-energized`, optional `SEQ_READY:<raw>`.

`PIN_DISARM` (16) is redundant on V2 and is not driven — arming is exactly what `test/dc_channel_test` does.

### 2.2 Direct solenoid / sequence

DC channels are the MCP23S17 outputs `ACTUATE1…16`; the single-hex-digit protocol reaches channels 1–15.

| Command | Effect |
|---|---|
| `S<chHex><state>` | Drive DC channel `chHex` (1–F) to `state` (0 or 1). Replies `Solenoid Command: <n> \| <state>`. **Rejected with `CMD_ERROR: chan <n> owned by BB`** if that channel's BB controller is active (SUSTAIN, AUTO_VENT, or ABORT). Manual control is available whenever that side's BB is off, and also on its press channel while latched in ABORT with no vent hardware configured. Malformed input replies `CMD_ERROR:short_packet` / `bad_channel` / `bad_state` / `chan_range`. |
| `s<chHex><state>.<5-digit-ms>,…` | Load a sequence (replaces any previous one). Replies one `SEQ_TOKEN idx=… chan=… state=… duration=…` per step, then `SEQ_ACK:count=<n>,raw=<cmd>`, or `SEQ_ERROR: Failed to parse sequence`. |
| `f` | Fire the loaded sequence. Replies `SEQ_EXEC_START:count=<n>,raw=<cmd>` then `Firing sequence!`, or `SEQ_ERROR: No sequence loaded`. Each step emits `SEQ_STEP:index=…,chan=…,state=…,duration=…`; completion emits `SEQ_EXEC_COMPLETE` and `SEQ_READY:<raw>`. |

As on V1, `S` and `f` are not gated on arm state in firmware; with `PIN_ARM` low the drive stage is unpowered.

### 2.3 Bang-bang configuration (persisted to EEPROM)

All three take effect immediately on the running controller and are saved to EEPROM. Any parse failure replies `BB_ERROR:parse` and no state changes.

| Command | Fields |
|---|---|
| `B<side><sp>,<db>,<wait>,<maxOpen>` | setpoint psi, symmetric deadband psi (>0), minimum closed dwell before reopening in ms (≤60000), slow-press max-open ms (≤60000; 0 = disabled). Closing is never delayed by `wait`. |
| `D<side><closeMs>` | Press-solenoid de-energize-to-mechanically-closed delay in ms (0–1000). Defaults to 15 ms. Persisted, but predictive cutoff stays runtime-disabled until explicitly enabled. |
| `V<side><trig>,<autoOn01>` | auto-vent trigger psi, auto-vent enable flag (0/1) |

Each emits `EVT:<ms>:CFG_PUSH:<side>:<k=v,...>` and rewrites the EEPROM block.

### 2.4 Bang-bang state commands

| Command | Effect | Preconditions |
|---|---|---|
| `b<side>1` | Enter `SUSTAIN`. | Armed, BB currently `DISABLED`, PT data fresh (`BB_ERROR:pt_stale`) and median filter full (`BB_ERROR:pt_settling`). |
| `b<side>0` | Leave `SUSTAIN` → `DISABLED`. Press closed. Vent untouched. **Also clears a latched `ABORT` without disarming** (operator acknowledge). | — |
| `e<side>1` | Enable predictive cutoff for this armed session. Emits `PRED_MODE`. Not persisted. | Armed. |
| `e<side>0` | Disable predictive cutoff. Rate calculation and `BBD:` telemetry continue. | Any. |
| `v<side>1` | **Manual vent**: press closed, vent open, state → `AUTO_VENT`. | Armed, vent DC channel configured, not in `ABORT`. |
| `v<side>0` | Exit `AUTO_VENT` → `DISABLED`. **Refused if pressure > deadband-high** — emits `EVT:…:AV_REJECT_CLOSE`. To override, disarm. | Only valid in `AUTO_VENT`. |
| `x<side>` | **Latched ABORT**: press closed, vent open. Cleared by `r` or `b<side>0`. | None. Safe from any state. |

If a vent channel is unset in `BoardConfig.h`, `v<side>1` and `x<side>` emit `EVT:…:AV_NO_HW`. Abort still closes the press valve.

### 2.5 Link heartbeat

| Command | Effect |
|---|---|
| `h` | Liveness only. No reply. The first `h` of a boot also **arms** the link watchdog (see [section 8](#8-gc-link-watchdog)). |

**GC must send `h` at 5 Hz for as long as it is connected.** Any other recognised command refreshes the watchdog too. Unrecognised lines (reply `CMD_ERROR:unknown`) do **not** — on V2 an idle bus can deliver noise as a "line", and that must not hold the watchdog open.

### 2.6 PT tare (persisted to EEPROM)

Offsets are subtracted after converting loop current to PSI. They affect the `P…` row and the bang-bang controllers only; the `p…` row stays the untared loop current.

| Command | Effect |
|---|---|
| `TL` | Tare LOX PT (ch 0) so the current reading reads 0 PSI. Requires fresh PT data. |
| `TF` | Tare Fuel PT (ch 1). |
| `Tz` | Clear all tare offsets. |
| `T<n>,<offset>` | Set explicit PSI offset for channel `n` (0 or 1). Example: `T0,12.345`. |

Each emits `EVT:…:PT_TARE:…` and saves to EEPROM. Parse failures reply `PT_ERROR:parse`; `TL`/`TF` without fresh data reply `PT_ERROR:no_data`. Taring restarts that channel's median filter, so `b…1` answers `pt_settling` for a moment afterwards.

## 3. Telemetry reference

### 3.1 DAQ rows (20 Hz, best-effort)

```
t<f0>,t<f1>,...,t<f15>\n     # 8 LC (raw V) + 8 TC (°C)
s<f0>,s<f1>,...,s<f15>\n     # Solenoid current (A), mux B
p<f0>,p<f1>,...,p<f15>\n     # PT loop current (mA), mux A
P<f0>,P<f1>\n                # BB PT pressure (PSI): scaled, tared, median-filtered
```

and at 2 Hz:

```
v<f0>,v<f1>,v<f2>\n          # INA230 bus voltages (U8, U10, U12)
```

All values are floats with 5 decimals; the identifier character repeats before every value. Rows are skipped whenever the UART lacks room plus a 256-byte reserve, so command replies and safety events never wait behind telemetry.

PSI conversion: `psi = (mA − 4) × PT_FULL_SCALE_PSI / 16 − tare`, then a 75-sample rolling median.

### 3.2 Bang-bang heartbeat (1 Hz, one per side)

```
BB:<side>:<state>:<press01>:<vent01>:<pressure_psi>
```

`<state>` is `OFF` / `SUS` / `AV` / `ABT`. `<pressure_psi>` is the live filtered reading, updated every tick regardless of arm or BB state.

### 3.2b Link watchdog status (1 Hz, right after the BB heartbeats)

```
LINK:<armed01>:<lost01>:<silent_ms>
```

| Field | Meaning |
|---|---|
| `armed01` | `1` once the watchdog has seen its first `h` this boot. **`0` means nothing is guarding the link.** |
| `lost01` | `1` while the link is timed out (stage 1 tripped). |
| `silent_ms` | ms since the last recognised command. `0` when un-armed. |

### 3.2c Predictive-close debug (10 Hz, one per side)

```
BBD:<side>:<state>:<press01>:<pressure_psi>:<rate_psi_s>:<projected_close_psi>:<deadband_high_psi>:<close_delay_ms>:<total_horizon_ms>:<rate_valid01>:<predictive_enabled01>
```

The rate updates once per completed PT sweep and is exponentially filtered (`BB_PRESSURE_RATE_ALPHA`). `total_horizon_ms = close_delay_ms + 0.5 × (PT_PSI_MEDIAN_WINDOW − 1) × measured_sample_interval_ms`. While rising, `projected_close_psi = pressure + rate × horizon`, and the press valve is de-energized when that reaches deadband-high. The ordinary close at deadband-high remains the fallback.

### 3.3 Audit events

```
EVT:<ms_uptime>:<category>:<side>:<detail>
```

Every bang-bang `detail` ends with `,pt=<psi>`.

| Category | Emitted when |
|---|---|
| `CFG_PUSH` | `B`/`D`/`V` accepted (also during EEPROM load at boot). |
| `BB_ON` / `BB_OFF` | Entered `SUSTAIN` / returned to `DISABLED`. |
| `VALVE` | Press or vent actually actuated: `press=N,reason=…` or `vent=N,reason=…`. |
| `PRED_MODE` | Predictive cutoff enabled/disabled (auto-disabled by every `forceSafe()`). |
| `PRED_CLOSE` | Predictive lookahead closed the press valve. Followed by the `VALVE` event. |
| `AV_ENTER` / `AV_EXIT` | Entered / left `AUTO_VENT`. |
| `AV_REJECT_CLOSE` | `v<side>0` refused, pressure above deadband-high. |
| `AV_NO_HW` | Vent-requiring command with no vent channel configured. |
| `ABORT_ENTER` / `ABORT_CLEAR` | Latched / cleared `ABORT`. |
| `SANITY_FAIL` | Filtered PT outside `BB_PRESSURE_MIN_PSI..MAX` while active. Auto-latches `ABORT`. |
| `PT_STALE` | No complete PT sweep for 50 ms while BB active (e.g. ADC1 hung). Force-safe; re-enable required. |
| `PT_TARE` | Tare offset changed. |
| `OWN_CONFLICT` | Command rejected by BB state (e.g. enable while not DISABLED). |
| `COMMS_WD_ARM` / `COMMS_LOSS` / `COMMS_DISARM` / `COMMS_OK` | Link watchdog armed / stage 1 / stage 2 / restored. Side is `-`. |

### 3.4 Boot lines

`BOOT`, optional `WARN:CRASH_DETECTED`, `EVT:…:CFG_PUSH` ×3 per side (if EEPROM valid), `PT_TARE:ch0=…,ch1=…`, `PANDA_V2_INIT`, any `WARN:ADC1_INIT_FAIL` / `ADC2_INIT_FAIL` / `PMONn_INIT_FAIL`, `Panda Initialized!`.

## 4. Hardware channel map (`include/BoardConfig.h`)

| Constant | Meaning | Current value |
|---|---|---|
| `BB_LOX_PT_CH` / `BB_FUEL_PT_CH` | mux A channel of the press-line PT | 0, 1 |
| `BB_LOX_DC_CH` / `BB_FUEL_DC_CH` | press-solenoid `ACTUATEn` | 1, 2 — **confirm against the harness** |
| `BB_LOX_VENT_DC_CH` / `BB_FUEL_VENT_DC_CH` | vent-solenoid `ACTUATEn` | 4, 7 |
| `BB_PRESSURE_MIN_PSI` / `MAX` | sanity bounds | −50 … 4000 |
| `BB_EEPROM_MAGIC` | bumped on `BBConfig` layout change | `0xBB46` |

## 5. Cold-start behavior

1. `PIN_ARM` driven low; all expander outputs off.
2. Both RS-485 links, ADCs, scanners, and power monitors initialized.
3. BB controllers bound to their I/O and `forceSafe()`d **before** `bbLoadEeprom()`, so config is restored but state is not.
4. PT tare offsets restored, `Panda Initialized!` sent.

The operator must issue `a` then `b<side>1` to start bang-bang. Predictive cutoff starts disabled every boot.

## 6. Safety-critical defaults

- **Disarm always wins** — `forceSafe()` on both sides unconditionally.
- **Predictive actuation defaults OFF and is never persisted.**
- **`ABORT` clears only on `r` or `b<side>0`.**
- **A single out-of-bounds filtered sample latches `ABORT`.**
- **The link watchdog is heartbeat-gated.** Check `LINK:<armed>` before every test.

## 7. Differences from V1

| | V1 | V2 |
|---|---|---|
| PT source | forwarded from V2 over crossover | local ADC1, mux A ch 0/1 at high priority |
| `p…` row | 2 values, shunt volts | 16 values, loop current mA |
| `s…` / `t…` rows | 12 values each | 16 values each |
| `v…` row | — | INA230 bus voltages, 2 Hz |
| DC outputs | Teensy GPIO | MCP23S17 `ACTUATE1…16` |
| Arm | ARM/DISARM relay pair | `PIN_ARM` level only |
| GC links | `Serial2` only | both RS-485 buses |
| Watchdog liveness | any line | any recognised command |
| Unknown command | ignored | `CMD_ERROR:unknown` |
| Mass-flow setpoint correction (`M`, `MDOT_*`) | present | removed |

## 8. GC link watchdog

GC only talks to the board when the operator acts, so silence is **not** evidence of a dead link — during a hold it is normal. Hence `h` at 5 Hz; `COMMS_LOSS_MS` (600 ms) is three missed beats.

The watchdog is **dormant at boot** and arms on the first `h`, latching on until reset. A GC that never sends `h` gets no protection — which is why `LINK:<armed>` is published every second.

| Elapsed silence | Constant | Action |
|---|---|---|
| ≥ 600 ms | `COMMS_LOSS_MS` | `COMMS_LOSS`. Both BB controllers `forceSafe()` every tick while lost. `ABORT` is exempt (its vent stays open). |
| ≥ 10 000 ms | `COMMS_DISARM_MS` | `COMMS_DISARM`. Identical to an operator `r`. |

Traffic resuming emits `COMMS_OK` and clears the latch. **It does not restart anything** — re-arm and re-enable.

What GC must implement:

1. Send `h\n` every 200 ms for as long as it is connected, regardless of arm state.
2. Stop the moment GC considers itself disconnected — a heartbeat that outlives the UI holds the watchdog open on a stand nobody is flying.
3. Show `LINK` `armed=0` as a warning and `lost=1` as an alarm.
4. Treat `COMMS_LOSS` / `COMMS_DISARM` as first-class alarms.
