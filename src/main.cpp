#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>

#include "BoardConfig.h"
#include "pins.h"

#include <INA230.h>
#include <MCP23S17.h>
#include <MCP3561RT.h>

#include "ArmingController.h"
#include "BangBang.h"
#include "CommsHandler.h"
#include "Scanner.h"
#include "SensorConfig.h"
#include "SequenceHandler.h"

/**
 * SERIAL PROTOCOL (RS-485, 460800 baud, both buses)
 *
 * See docs/GC_USERS_GUIDE.md for the canonical command + telemetry reference.
 * Both RS-485 buses are GC links: commands are accepted on either, replies go
 * back on the bus the command arrived on, and telemetry + EVT: lines go out
 * on both.
 *
 * Commands:
 *   'a'                                 Arm (PIN_ARM held high)
 *   'r'                                 Disarm (also forceSafe()s BB)
 *   'S<chHex><state>'                   Direct solenoid command
 *   's<chHex><state>.<ms 5-digit>,...'  Load sequence
 *   'f'                                 Fire loaded sequence
 *   'B<side><sp>,<db>,<wait>,<maxOpen>' Configure BB core (L or F)
 *   'D<side><closeMs>'                  Configure predictive close delay
 *   'V<side><trig>,<autoOn01>'          Configure BB auto-vent
 *   'b<side><0|1>'                      Enable/disable bang-bang control
 *   'e<side><0|1>'                      Enable/disable predictive cutoff
 *   'v<side><0|1>'                      Manual vent open(1) / close(0)
 *   'x<side>'                           Latched abort (cleared by 'r' or 'b<side>0')
 *   'h'                                 GC heartbeat (see link watchdog below)
 *   'TL' / 'TF'                         Tare LOX/Fuel PT to current PSI (persisted)
 *   'Tz'                                Clear all PT tare offsets
 *   'T<n>,<offset>'                     Set explicit PSI offset for channel n
 *
 * Telemetry:
 *   't...'  load cells, raw V (NUM_LC_CH)            20 Hz
 *   's...'  solenoid current, A (NUM_MUX_B_CH)       20 Hz
 *   'p...'  PT loop current, mA (NUM_PT_CH)          20 Hz
 *   'P...'  BB PT pressure, PSI (NUM_PSI_PT_CH)      20 Hz
 *   'v...'  INA230 bus voltages                       2 Hz
 *   'BB:<L|F>:<state>:<press>:<vent>:<pressure>'      1 Hz
 *   'LINK:<armed>:<lost>:<silentMs>'                  1 Hz
 *   'BBD:<side>:...'                                 10 Hz predictive debug
 *   'EVT:<ms>:<cat>:<L|F|->:<detail>'                 on every BB transition
 *
 * GC LINK WATCHDOG
 * GC must send 'h' at 5 Hz. Any recognised command counts as liveness; 'h'
 * exists so GC can prove liveness without commanding anything. After
 * COMMS_LOSS_MS of silence both bang-bang controllers are forced safe; after
 * COMMS_DISARM_MS the board disarms itself exactly as if 'r' had been
 * received. The watchdog is dormant until the first 'h' of the boot — watch
 * LINK:<armed> to confirm it is active. Recovery is never automatic.
 */

// ── Hardware instances ──────────────────────────────────────────────

MCP3561RT adc1(PIN_ADC1_CS, PIN_ADC1_IRQ, SPI, SPI_ADC_SETTINGS, ADC1_VREF_V);
MCP3561RT adc2(PIN_ADC2_CS, PIN_ADC2_IRQ, SPI1, SPI_BUS1_SETTINGS, ADC2_VREF_V);

INA230 pmon0(Wire, INA230_ADDR_U8);
INA230 pmon1(Wire, INA230_ADDR_U10);
INA230 pmon2(Wire, INA230_ADDR_U12);

static bool pmonOk[3] = {false, false, false};

// RS-485 comms — Serial7 = bus 1 (pins 28/29), Serial6 = bus 2 (pins 25/24)
CommsHandler comms(Serial7, PIN_RS485_1_DE);
CommsHandler comms2(Serial6, PIN_RS485_2_DE);

ArmingController arming(PIN_ARM);

// Solenoid drive — MCP23S17 U35, SPI1 (LPSPI3) alongside ADC2
MCP23S17 ioexp(PIN_IOEXP_CS, SPI1, SPI_BUS1_SETTINGS, IOEXP_HW_ADDR);
SequenceHandler seq(ioexp);

// ── Scanner output buffers ──────────────────────────────────────────

float muxA_data[NUM_MUX_A_CH] = {0};
float muxB_data[NUM_MUX_B_CH] = {0};
float muxC_data[NUM_MUX_C_CH] = {0};

float ptData[NUM_PT_CH] = {0};
float lcData[NUM_LC_CH] = {0};
float curData[NUM_MUX_B_CH] = {0};

MuxBank adc1Banks[] = {
    {{PIN_MUX_A_S0, PIN_MUX_A_S1, PIN_MUX_A_S2, PIN_MUX_A_S3},
     NUM_MUX_A_CH,
     MCP3561RT::Mux::CH0,
     muxA_data},
    {{PIN_MUX_B_S0, PIN_MUX_B_S1, PIN_MUX_B_S2, PIN_MUX_B_S3},
     NUM_MUX_B_CH,
     MCP3561RT::Mux::CH1,
     muxB_data},
};

MuxBank adc2Banks[] = {
    {{PIN_MUX_C_S0, PIN_MUX_C_S1, PIN_MUX_C_S2, PIN_MUX_C_S3},
     MUX_C_LC_START + MUX_C_LC_COUNT, // only the load-cell channels
     MCP3561RT::Mux::CH0,
     muxC_data},
};

// The PSI-view PTs (mux A bank 0, channels 0..NUM_PSI_PT_CH-1) are sampled
// between every other ADC1 channel so bang-bang sees fresh pressure at a few
// hundred Hz. One completed pass over this list is one "PT frame".
static const ScanSlot adc1FastSlots[NUM_PSI_PT_CH] = {{0, 0}, {0, 1}};
static_assert(NUM_PSI_PT_CH == 2, "update adc1FastSlots when changing NUM_PSI_PT_CH");

Scanner scanner1(adc1, adc1Banks, 2, adc1FastSlots, NUM_PSI_PT_CH);
Scanner scanner2(adc2, adc2Banks, 1);

// ── PT pressure view (filled on every completed fast PT sweep) ──────

static float ptPsiData[NUM_PSI_PT_CH] = {0};
static uint32_t lastPtMs = 0;
static uint32_t lastPtSweep = 0;
static bool hasValidPt = false;

// ── GC link health ──────────────────────────────────────────────────
// lastGcRxMs is refreshed by ANY recognised command from GC, not just
// heartbeats: a command is proof of life too. gcWatchdogArmed latches on the
// first 'h' of the boot and never clears, so a GC that stops heartbeating
// after having started cannot silently switch the protection back off.
static uint32_t lastGcRxMs = 0;
static bool gcWatchdogArmed = false;
static bool gcLinkLost = false;   // stage 1 tripped, not yet recovered
static bool gcDisarmDone = false; // stage 2 fired for this outage

// ── Bang-bang controllers ───────────────────────────────────────────
// BB configuration and safety limits are in PSI, so controllers consume the
// scaled view rather than the raw mA telemetry values.
BBController bbLox(ptPsiData, BB_LOX_PT_CH, BB_LOX_DC_CH, BB_LOX_VENT_DC_CH,
                   'L');
BBController bbFuel(ptPsiData, BB_FUEL_PT_CH, BB_FUEL_DC_CH, BB_FUEL_VENT_DC_CH,
                    'F');

// ── Output helpers ──────────────────────────────────────────────────

static void broadcastLine(const char *msg) {
  comms.sendLine(msg);
  comms2.sendLine(msg);
}

// BB → DC channel setter
static bool bbSetChannel(uint8_t ch1, bool state) {
  return seq.setChannel(ch1, state);
}

// BB → audit-event emitter. Format: EVT:<ms>:<cat>:<side>:<detail>
static void bbEmit(const char *cat, char side, const char *detail) {
  char buf[160];
  snprintf(buf, sizeof(buf), "EVT:%lu:%s:%c:%s", (unsigned long)millis(), cat,
           side, detail ? detail : "");
  broadcastLine(buf);
}

// Manual S commands are blocked on channels BB is actively driving.
static bool isBbOwned(uint8_t ch1) {
  return bbLox.ownsChannel(ch1) || bbFuel.ownsChannel(ch1);
}

// ── PT tare (persisted) ─────────────────────────────────────────────

static float ptTarePsiOffset[NUM_PSI_PT_CH] = {0};

struct PtTareEepromBlock {
  uint16_t magic;
  float offset_psi[NUM_PSI_PT_CH];
  uint8_t crc;
};

static uint8_t ptTareComputeCrc(const PtTareEepromBlock &block) {
  uint8_t crc = 0;
  const uint8_t *p = reinterpret_cast<const uint8_t *>(block.offset_psi);
  for (size_t i = 0; i < sizeof(block.offset_psi); i++)
    crc ^= p[i];
  return crc;
}

static void ptTareLoadEeprom() {
  PtTareEepromBlock block;
  EEPROM.get(PT_TARE_EEPROM_ADDR, block);
  if (block.magic != PT_TARE_EEPROM_MAGIC)
    return;
  if (block.crc != ptTareComputeCrc(block))
    return;
  memcpy(ptTarePsiOffset, block.offset_psi, sizeof(ptTarePsiOffset));
}

static void ptTareSaveEeprom() {
  PtTareEepromBlock block;
  block.magic = PT_TARE_EEPROM_MAGIC;
  memcpy(block.offset_psi, ptTarePsiOffset, sizeof(block.offset_psi));
  block.crc = ptTareComputeCrc(block);
  EEPROM.put(PT_TARE_EEPROM_ADDR, block);
}

static float ptMaToPsi(float currentMa) {
  return (currentMa - PT_ZERO_MA) * (PT_FULL_SCALE_PSI / PT_SPAN_MA);
}

static bool hasFreshPt() {
  return hasValidPt && (millis() - lastPtMs <= BB_PT_STALE_MS);
}

// ── PSI median filter ───────────────────────────────────────────────
// Rejects sustained switching noise (e.g. solenoid/arm-line coupling into the
// PT analog front-end) without hiding a genuine, continuously out-of-range
// fault — BB's sanity check still sees the filtered value, so a real fault
// still trips it. Raw ptData[] is left unfiltered so GC keeps the true signal.
struct PsiMedianFilter {
  float ring[PT_PSI_MEDIAN_WINDOW];
  uint8_t count = 0;
  uint8_t idx = 0;
};

static PsiMedianFilter ptPsiFilters[NUM_PSI_PT_CH];

static void resetMedianFilter(uint8_t ch) { ptPsiFilters[ch] = PsiMedianFilter{}; }

static bool ptPsiSettled() {
  if (!hasFreshPt())
    return false;
  for (uint8_t ch = 0; ch < NUM_PSI_PT_CH; ch++) {
    if (ptPsiFilters[ch].count < PT_PSI_MEDIAN_WINDOW)
      return false;
  }
  return true;
}

static float medianFilterPsi(uint8_t ch, float sample) {
  PsiMedianFilter &f = ptPsiFilters[ch];

  f.ring[f.idx] = sample;
  f.idx = (f.idx + 1) % PT_PSI_MEDIAN_WINDOW;
  if (f.count < PT_PSI_MEDIAN_WINDOW)
    f.count++;

  float sorted[PT_PSI_MEDIAN_WINDOW];
  const uint8_t n = f.count;
  for (uint8_t i = 0; i < n; i++) {
    const uint8_t ri =
        (f.idx + PT_PSI_MEDIAN_WINDOW - n + i) % PT_PSI_MEDIAN_WINDOW;
    sorted[i] = f.ring[ri];
  }

  for (uint8_t i = 1; i < n; i++) {
    const float key = sorted[i];
    int8_t j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }
  return sorted[n / 2];
}

// Commit a new PT frame once the scanner has converted every fast PT slot.
// A pass with a failed/timed-out conversion does not advance fastSweepCount(),
// so a hung ADC1 stops refreshing lastPtMs and the stale guard trips.
static void updatePtFrame() {
  const uint32_t sweep = scanner1.fastSweepCount();
  if (sweep == lastPtSweep)
    return;
  lastPtSweep = sweep;

  float psi[NUM_PSI_PT_CH];
  for (uint8_t ch = 0; ch < NUM_PSI_PT_CH; ch++) {
    const float ma = convertPT(muxA_data[ch], ch);
    if (!isfinite(ma))
      return;
    psi[ch] = ptMaToPsi(ma) - ptTarePsiOffset[ch];
  }
  for (uint8_t ch = 0; ch < NUM_PSI_PT_CH; ch++)
    ptPsiData[ch] = medianFilterPsi(ch, psi[ch]);

  lastPtMs = millis();
  hasValidPt = true;
}

// ── Disarm (operator 'r' and watchdog stage 2 share this path) ──────

static void disarmAll() {
  arming.disarm();
  bbLox.forceSafe();
  bbFuel.forceSafe();
  seq.cancelExecution();
  seq.setAllOff();
}

// ── Sequence reporting ──────────────────────────────────────────────

static void onSeqStep(uint8_t index, const SequenceStep &step) {
  char buf[96];
  snprintf(buf, sizeof(buf), "SEQ_STEP:index=%u,chan=%u,state=%s,duration=%lu",
           index, step.channel, step.state ? "ON" : "OFF",
           (unsigned long)step.delay_ms);
  broadcastLine(buf);
}

static void sendSeqReady(CommsHandler &out) {
  if (!seq.hasSequence())
    return;
  char buf[280];
  snprintf(buf, sizeof(buf), "SEQ_READY:%s", seq.getLastCommand());
  out.sendLine(buf);
}

// ── BB command dispatch ─────────────────────────────────────────────

static BBController *pickSide(char c) {
  if (c == 'L')
    return &bbLox;
  if (c == 'F')
    return &bbFuel;
  return nullptr;
}

// Shared preamble: length + side check. Returns nullptr after replying.
static BBController *bbTarget(const char *pkt, size_t minLen,
                              CommsHandler &out) {
  if (strlen(pkt) < minLen) {
    out.sendLine("BB_ERROR:short");
    return nullptr;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl)
    out.sendLine("BB_ERROR:bad_side");
  return ctrl;
}

static void handleB(const char *pkt, CommsHandler &out) { // core config
  BBController *ctrl = bbTarget(pkt, 4, out);
  if (!ctrl)
    return;
  float sp, db;
  unsigned long wt, maxOpen;
  if (sscanf(pkt + 2, "%f,%f,%lu,%lu", &sp, &db, &wt, &maxOpen) != 4 ||
      sp < 0.0f || db <= 0.0f || wt > 60000UL || maxOpen > 60000UL) {
    out.sendLine("BB_ERROR:parse");
    return;
  }
  ctrl->configureCore(sp, db, (uint32_t)wt, (uint32_t)maxOpen);
  bbSaveEeprom(bbLox, bbFuel);
}

static void handleV(const char *pkt, CommsHandler &out) { // auto-vent config
  BBController *ctrl = bbTarget(pkt, 4, out);
  if (!ctrl)
    return;
  float trig;
  int autoOn;
  if (sscanf(pkt + 2, "%f,%d", &trig, &autoOn) != 2 ||
      (autoOn != 0 && autoOn != 1)) {
    out.sendLine("BB_ERROR:parse");
    return;
  }
  ctrl->configureVent(trig, autoOn != 0);
  bbSaveEeprom(bbLox, bbFuel);
}

static void handleD(const char *pkt, CommsHandler &out) { // predictive delay
  BBController *ctrl = bbTarget(pkt, 3, out);
  if (!ctrl)
    return;
  unsigned long closeDelayMs;
  char trailing;
  if (sscanf(pkt + 2, "%lu%c", &closeDelayMs, &trailing) != 1 ||
      closeDelayMs > 1000UL) {
    out.sendLine("BB_ERROR:parse");
    return;
  }
  ctrl->configurePredictiveClose((uint32_t)closeDelayMs);
  bbSaveEeprom(bbLox, bbFuel);
}

static void handleLowerB(const char *pkt, CommsHandler &out) { // sustain on/off
  BBController *ctrl = bbTarget(pkt, 3, out);
  if (!ctrl)
    return;
  char stCh = pkt[2];
  if (stCh == '1') {
    if (!arming.isArmed()) {
      out.sendLine("BB_ERROR:not_armed");
      return;
    }
    if (!hasFreshPt()) {
      out.sendLine("BB_ERROR:pt_stale");
      return;
    }
    if (!ptPsiSettled()) {
      out.sendLine("BB_ERROR:pt_settling");
      return;
    }
    ctrl->enableSustain();
  } else if (stCh == '0') {
    ctrl->disableSustain();
  } else {
    out.sendLine("BB_ERROR:bad_arg");
  }
}

static void handleLowerE(const char *pkt, CommsHandler &out) { // predictive
  if (strlen(pkt) != 3) {
    out.sendLine("BB_ERROR:parse");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    out.sendLine("BB_ERROR:bad_side");
    return;
  }
  if (pkt[2] == '1') {
    if (!arming.isArmed()) {
      out.sendLine("BB_ERROR:not_armed");
      return;
    }
    ctrl->setPredictiveEnabled(true);
  } else if (pkt[2] == '0') {
    ctrl->setPredictiveEnabled(false);
  } else {
    out.sendLine("BB_ERROR:bad_arg");
  }
}

static void handleLowerV(const char *pkt, CommsHandler &out) { // manual vent
  BBController *ctrl = bbTarget(pkt, 3, out);
  if (!ctrl)
    return;
  char stCh = pkt[2];
  if (stCh == '1') {
    if (!arming.isArmed()) {
      out.sendLine("BB_ERROR:not_armed");
      return;
    }
    ctrl->manualVent();
  } else if (stCh == '0') {
    ctrl->manualVentClose(false);
  } else {
    out.sendLine("BB_ERROR:bad_arg");
  }
}

static void handleLowerX(const char *pkt, CommsHandler &out) { // latched abort
  BBController *ctrl = bbTarget(pkt, 2, out);
  if (!ctrl)
    return;
  ctrl->latchAbort();
}

static void applyPtTareOffset(uint8_t ch, float offsetPsi, char side) {
  ptTarePsiOffset[ch] = offsetPsi;
  resetMedianFilter(ch);
  ptTareSaveEeprom();

  char detail[48];
  snprintf(detail, sizeof(detail), "ch=%u,offset=%.3f", ch, offsetPsi);
  bbEmit("PT_TARE", side, detail);
}

static void handleT(const char *pkt, CommsHandler &out) {
  const char *body = pkt + 1;
  if (body[0] == 'z' && body[1] == '\0') {
    memset(ptTarePsiOffset, 0, sizeof(ptTarePsiOffset));
    for (uint8_t ch = 0; ch < NUM_PSI_PT_CH; ch++)
      resetMedianFilter(ch);
    ptTareSaveEeprom();
    bbEmit("PT_TARE", '-', "clear all");
    return;
  }

  if ((body[0] == 'L' || body[0] == 'F') && body[1] == '\0') {
    if (!hasFreshPt()) {
      out.sendLine("PT_ERROR:no_data");
      return;
    }
    const uint8_t ch = (body[0] == 'L') ? BB_LOX_PT_CH : BB_FUEL_PT_CH;
    // Tare against the untared, unfiltered reading so the new offset zeroes
    // the present pressure exactly.
    applyPtTareOffset(ch, ptMaToPsi(convertPT(muxA_data[ch], ch)), body[0]);
    return;
  }

  unsigned ch = 0;
  float offsetPsi = 0.0f;
  if (sscanf(body, "%u,%f", &ch, &offsetPsi) != 2 || ch >= NUM_PSI_PT_CH ||
      !isfinite(offsetPsi)) {
    out.sendLine("PT_ERROR:parse");
    return;
  }

  char side = '-';
  if (ch == BB_LOX_PT_CH)
    side = 'L';
  else if (ch == BB_FUEL_PT_CH)
    side = 'F';
  applyPtTareOffset((uint8_t)ch, offsetPsi, side);
}

// ── Command handling ────────────────────────────────────────────────
// Receives a packet and the bus it arrived on so replies go back on the same
// bus. Returns true if the packet was a recognised command (liveness).

static bool handleCommand(char *packet, CommsHandler &out) {
  if (!packet || packet[0] == '\0')
    return false;

  switch (packet[0]) {

  case 'a':
    arming.arm();
    out.sendLine("Arming!");
    break;

  case 'r':
    disarmAll();
    out.sendLine("Disarming!");
    out.sendLine("SEQ_ABORT: Outputs de-energized");
    sendSeqReady(out);
    break;

  case 's': {
    if (!seq.setCommand(packet)) {
      out.sendLine("SEQ_ERROR: Failed to parse sequence");
      break;
    }
    char buf[300];
    for (uint8_t i = 0; i < seq.getNumSteps(); i++) {
      const SequenceStep &st = seq.getStep(i);
      snprintf(buf, sizeof(buf),
               "SEQ_TOKEN idx=%u chan=%u state=%s duration=%lu", i, st.channel,
               st.state ? "ON" : "OFF", (unsigned long)st.delay_ms);
      out.sendLine(buf);
    }
    snprintf(buf, sizeof(buf), "SEQ_ACK:count=%u,raw=%s", seq.getNumSteps(),
             seq.getLastCommand());
    out.sendLine(buf);
    break;
  }

  case 'S': {
    if (strlen(packet) < 3) {
      out.sendLine("CMD_ERROR:short_packet");
      break;
    }
    unsigned chan = 0;
    char ch = packet[1];
    if (ch >= '0' && ch <= '9')
      chan = ch - '0';
    else if (ch >= 'A' && ch <= 'F')
      chan = 10 + (ch - 'A');
    else if (ch >= 'a' && ch <= 'f')
      chan = 10 + (ch - 'a');
    else {
      out.sendLine("CMD_ERROR:bad_channel");
      break;
    }
    unsigned state = packet[2] - '0';
    if (state > 1) {
      out.sendLine("CMD_ERROR:bad_state");
      break;
    }
    if (chan < 1 || chan > NUM_ACTUATORS) {
      out.sendLine("CMD_ERROR:chan_range");
      break;
    }

    char buf[64];
    if (isBbOwned(chan)) {
      snprintf(buf, sizeof(buf), "CMD_ERROR: chan %u owned by BB", chan);
    } else {
      seq.setChannel(chan, state);
      snprintf(buf, sizeof(buf), "Solenoid Command: %u | %u", chan, state);
    }
    out.sendLine(buf);
    break;
  }

  case 'f': {
    if (!seq.hasSequence()) {
      out.sendLine("SEQ_ERROR: No sequence loaded");
      break;
    }
    char buf[300];
    snprintf(buf, sizeof(buf), "SEQ_EXEC_START:count=%u,raw=%s",
             seq.getNumSteps(), seq.getLastCommand());
    out.sendLine(buf);
    seq.execute();
    out.sendLine("Firing sequence!");
    break;
  }

  case 'B': handleB(packet, out); break;
  case 'D': handleD(packet, out); break;
  case 'V': handleV(packet, out); break;
  case 'b': handleLowerB(packet, out); break;
  case 'e': handleLowerE(packet, out); break;
  case 'v': handleLowerV(packet, out); break;
  case 'x': handleLowerX(packet, out); break;
  case 'T': handleT(packet, out); break;

  case ID_GC_HEARTBEAT:
    // Liveness only, and deliberately silent: at 5 Hz an acknowledgement per
    // beat would put avoidable traffic on the bus. The 1 Hz LINK: line is the
    // acknowledgement.
    if (!gcWatchdogArmed) {
      gcWatchdogArmed = true;
      bbEmit("COMMS_WD_ARM", '-', "GC heartbeat seen; link watchdog active");
    }
    break;

  default:
    // Unrecognised lines (including line noise on an idle bus) do not count
    // as proof of GC liveness.
    out.sendLine("CMD_ERROR:unknown");
    return false;
  }
  return true;
}

// ── GC link watchdog ────────────────────────────────────────────────
// Two stages:
//   stage 1 (COMMS_LOSS_MS)   bang-bang control off, BB valves driven closed
//   stage 2 (COMMS_DISARM_MS) full disarm, identical to operator 'r'
//
// Recovery never restarts control on its own. A healthy link coming back only
// clears the latch; the operator must re-arm and re-issue b<side>1.
static void serviceGcLinkWatchdog(uint32_t now) {
  if (!gcWatchdogArmed)
    return;

  const uint32_t silentMs = now - lastGcRxMs;

  if (silentMs < COMMS_LOSS_MS) {
    if (gcLinkLost) {
      gcLinkLost = false;
      gcDisarmDone = false;
      bbEmit("COMMS_OK", '-', "GC link restored; re-arm and re-enable to resume");
    }
    return;
  }

  if (!gcLinkLost) {
    gcLinkLost = true;
    bbEmit("COMMS_LOSS", '-', "GC link silent; bang-bang forced safe");
  }

  // forceSafe() is a no-op once valves are already closed, so calling it every
  // tick costs nothing. ABORT is exempt: it parks the vent OPEN, which is the
  // safe state for the over-pressure that latched it.
  if (bbLox.state() != BBState::ABORT)
    bbLox.forceSafe();
  if (bbFuel.state() != BBState::ABORT)
    bbFuel.forceSafe();

  if (silentMs >= COMMS_DISARM_MS && !gcDisarmDone) {
    gcDisarmDone = true;
    bbEmit("COMMS_DISARM", '-', "no GC link for 10s; disarming");
    disarmAll();
  }
}

// ── Sensor conversion ───────────────────────────────────────────────

static void updateConversions() {
  for (uint8_t i = 0; i < NUM_PT_CH; i++)
    ptData[i] = convertPT(muxA_data[i], i);

  for (uint8_t i = 0; i < NUM_MUX_B_CH; i++)
    curData[i] = convertCurrent(muxB_data[i]);

  for (uint8_t i = 0; i < NUM_LC_CH; i++)
    lcData[i] = convertLC(muxC_data[MUX_C_LC_START + i], i);
}

// ── Telemetry ───────────────────────────────────────────────────────
// Periodic telemetry is best-effort. Unlike command responses and BB events,
// it must never block the main loop waiting for UART buffer space: doing that
// can prevent us from reading the disable/disarm command which clears a live
// control state.

static bool tryWriteRows(CommsHandler &out, const char *const *rows,
                         size_t numRows, size_t extraReserve = 0) {
  size_t frameLen = 0;
  for (size_t i = 0; i < numRows; i++)
    frameLen += strlen(rows[i]);
  const int available = out.availableForWrite();
  if (available < 0 ||
      (size_t)available < frameLen + TX_PRIORITY_RESERVE + extraReserve)
    return false;
  for (size_t i = 0; i < numRows; i++)
    out.send(rows[i]);
  return true;
}

static void sendTelemetry() {
  static char lcRow[256], sRow[512], pRow[512], psiRow[128];

  CommsHandler::toCSVRow(lcData, ID_LC, NUM_LC_CH, lcRow, sizeof(lcRow));
  CommsHandler::toCSVRow(curData, ID_SOLENOID_CURRENT, NUM_MUX_B_CH, sRow,
                         sizeof(sRow));
  CommsHandler::toCSVRow(ptData, ID_PT, NUM_PT_CH, pRow, sizeof(pRow));
  CommsHandler::toCSVRow(ptPsiData, ID_PT_PSI, NUM_PSI_PT_CH, psiRow,
                         sizeof(psiRow));

  const char *rows[] = {lcRow, sRow, pRow, psiRow};
  tryWriteRows(comms, rows, 4);
  tryWriteRows(comms2, rows, 4);
}

static void sendPowerTelemetry() {
  char buf[128];
  INA230 *pmons[3] = {&pmon0, &pmon1, &pmon2};
  float voltages[3] = {0};
  for (int i = 0; i < 3; i++) {
    if (pmonOk[i])
      voltages[i] = pmons[i]->busVoltage_V();
  }
  CommsHandler::toCSVRow(voltages, ID_POWER, 3, buf, sizeof(buf));
  const char *rows[] = {buf};
  tryWriteRows(comms, rows, 1);
  tryWriteRows(comms2, rows, 1);
}

static const char *stateStr(BBState s) {
  switch (s) {
  case BBState::DISABLED:
    return "OFF";
  case BBState::SUSTAIN:
    return "SUS";
  case BBState::AUTO_VENT:
    return "AV";
  case BBState::ABORT:
    return "ABT";
  }
  return "??";
}

// BB:<side>:<state>:<press01>:<vent01>:<pressure_psi>
static void formatBbHeartbeat(const BBController &c, char *buf, size_t len) {
  snprintf(buf, len, "BB:%c:%s:%d:%d:%.1f\n", c.busId(), stateStr(c.state()),
           c.isPressOpen() ? 1 : 0, c.isVentOpen() ? 1 : 0, c.lastPressure());
}

// BBD:<side>:<state>:<press01>:<pressure_psi>:<rate_psi_s>:
//     <projected_close_psi>:<deadband_high_psi>:<close_delay_ms>:
//     <total_horizon_ms>:<rate_valid01>:<predictive_enabled01>
static void formatBbDebug(const BBController &c, char *buf, size_t len) {
  const float hi = c.config().setpoint_psi + c.config().deadband_psi * 0.5f;
  snprintf(buf, len, "BBD:%c:%s:%d:%.2f:%.2f:%.2f:%.2f:%lu:%.1f:%d:%d\n",
           c.busId(), stateStr(c.state()), c.isPressOpen() ? 1 : 0,
           c.lastPressure(), c.pressureRate(), c.projectedPressure(), hi,
           (unsigned long)c.config().close_delay_ms, c.predictionHorizonMs(),
           c.pressureRateValid() ? 1 : 0, c.predictiveEnabled() ? 1 : 0);
}

// LINK:<armed01>:<lost01>:<silent_ms>
// <armed01> = 0 means GC has never sent a heartbeat this boot and NOTHING is
// guarding the link.
static void formatLinkStatus(uint32_t now, char *buf, size_t len) {
  snprintf(buf, len, "LINK:%d:%d:%lu\n", gcWatchdogArmed ? 1 : 0,
           gcLinkLost ? 1 : 0,
           gcWatchdogArmed ? (unsigned long)(now - lastGcRxMs) : 0UL);
}

static void sendBbHeartbeat(uint32_t now) {
  char lox[64], fuel[64], link[48];
  formatBbHeartbeat(bbLox, lox, sizeof(lox));
  formatBbHeartbeat(bbFuel, fuel, sizeof(fuel));
  formatLinkStatus(now, link, sizeof(link));
  const char *rows[] = {lox, fuel, link};
  tryWriteRows(comms, rows, 3);
  tryWriteRows(comms2, rows, 3);
}

static void sendBbDebug() {
  char lox[128], fuel[128];
  formatBbDebug(bbLox, lox, sizeof(lox));
  formatBbDebug(bbFuel, fuel, sizeof(fuel));
  const char *rows[] = {lox, fuel};
  tryWriteRows(comms, rows, 2);
  tryWriteRows(comms2, rows, 2);
}

// ── Sequence completion reporting ───────────────────────────────────

static bool seqWasActive = false;

static void checkSequenceComplete() {
  bool active = seq.isActive();
  if (seqWasActive && !active) {
    broadcastLine("SEQ_EXEC_COMPLETE");
    sendSeqReady(comms);
    sendSeqReady(comms2);
  }
  seqWasActive = active;
}

// ── Arduino entry points ────────────────────────────────────────────

static void broadcastWarn(bool ok, const char *msg) {
  if (!ok)
    broadcastLine(msg);
}

void setup() {
  Serial.begin(DEBUG_BAUD);

  if (CrashReport) {
    Serial.print(CrashReport);
    // Give time for USB serial to flush before continuing
    delay(2000);
  }

  // Arm line low before anything else can drive a solenoid.
  arming.begin();

  // Park every chip select HIGH before any bus traffic. Teensy pins boot as
  // floating inputs, and a device's /CS only becomes a driven output when that
  // device's begin() runs. On SPI1 two devices share the bus, so whichever one
  // initializes second has a floating /CS during the first one's init and can
  // decode that traffic as its own.
  for (uint8_t cs : {PIN_ADC1_CS, PIN_ADC2_CS, PIN_IOEXP_CS}) {
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
  }

  SPI.begin();
  SPI1.setMISO(PIN_SPI1_MISO); // default is pin 1; hardware uses pin 39
  SPI1.begin();

  // DC solenoid outputs via the MCP23S17 — brought up before the ADCs so its
  // /CS is driven high and all outputs are off (matches dc_channel_test).
  seq.begin();
  seq.setStepCallback(onSeqStep);

  Wire.begin();
  Wire.setTimeout(2500); // 2.5ms max per I2C transaction (default is infinite)

  // Bring up both RS-485 buses immediately — any TX problem shows up on BOOT
  comms.begin(RS485_BAUD);
  comms2.begin(RS485_BAUD);

  // V2 PCB has TX differential pair (Y/Z) swapped on the RJ45.
  // RX pair (A/B) is correct — do NOT set RXINV.
  // TODO: fix Y/Z routing in next board revision
  //
  // Bus 1 (Serial7 → LPUART7)
  LPUART7_CTRL |= LPUART_CTRL_TXINV;
  // Bus 2 (Serial6 → LPUART1)
  LPUART1_CTRL |= LPUART_CTRL_TXINV;

  broadcastLine("BOOT");
  if (CrashReport)
    broadcastLine("WARN:CRASH_DETECTED");

  bool adc1_ok = adc1.begin();
  bool adc2_ok = adc2.begin();

  scanner1.begin();
  scanner2.begin();

  pmonOk[0] = pmon0.begin(0.1f, 5.0f);
  pmonOk[1] = pmon1.begin(0.1f, 5.0f);
  pmonOk[2] = pmon2.begin(0.1f, 5.0f);

  // Wire BB IO and load persisted config. Controllers come up DISABLED
  // regardless of what EEPROM contained — config is restored but the state
  // machine starts safe.
  bbLox.bindIO(bbSetChannel, bbEmit);
  bbFuel.bindIO(bbSetChannel, bbEmit);
  bbLox.forceSafe();
  bbFuel.forceSafe();
  bbLoadEeprom(bbLox, bbFuel);
  ptTareLoadEeprom();

  char buf[64];
  snprintf(buf, sizeof(buf), "PT_TARE:ch0=%.3f,ch1=%.3f", ptTarePsiOffset[0],
           ptTarePsiOffset[1]);
  broadcastLine(buf);

  broadcastLine("PANDA_V2_INIT");
  broadcastWarn(adc1_ok, "WARN:ADC1_INIT_FAIL");
  broadcastWarn(adc2_ok, "WARN:ADC2_INIT_FAIL");
  broadcastWarn(pmonOk[0], "WARN:PMON0_INIT_FAIL");
  broadcastWarn(pmonOk[1], "WARN:PMON1_INIT_FAIL");
  broadcastWarn(pmonOk[2], "WARN:PMON2_INIT_FAIL");
  broadcastLine("Panda Initialized!");

  Serial.println("PandaV2 ready");
}

void loop() {
  // 1. Drain all queued commands. Any recognised line proves GC and the wire
  //    are alive — refresh before dispatch so even a rejected command counts.
  CommsHandler *buses[] = {&comms, &comms2};
  for (CommsHandler *bus : buses) {
    for (int i = 0; i < NUM_MAX_COMMANDS; i++) {
      bus->poll();
      if (!bus->isPacketReady())
        break;
      char *packet = bus->takePacket();
      const uint32_t rxMs = millis();
      if (handleCommand(packet, *bus))
        lastGcRxMs = rxMs;
    }
  }

  // 2. Sequencer
  seq.update();

  // 3. ADC scanning + conversions
  scanner1.update();
  scanner2.update();
  updateConversions();
  updatePtFrame();

  // 4. Link watchdog — serviced before BB runs, so no controller actuates on
  //    this tick on the strength of a command from a link that is already gone.
  serviceGcLinkWatchdog(millis());

  // 5. Bang-bang. Loss of PT data must not leave a valve controlled from a
  //    stale sample. Fresh data does not auto-restart control; the operator
  //    must explicitly enable sustain again. ABORT stays latched.
  if (!hasFreshPt()) {
    if (bbLox.state() == BBState::SUSTAIN ||
        bbLox.state() == BBState::AUTO_VENT) {
      bbEmit("PT_STALE", 'L', "PT timeout; forcing safe");
      bbLox.forceSafe();
    }
    if (bbFuel.state() == BBState::SUSTAIN ||
        bbFuel.state() == BBState::AUTO_VENT) {
      bbEmit("PT_STALE", 'F', "PT timeout; forcing safe");
      bbFuel.forceSafe();
    }
  }
  const uint32_t pressureSampleMs = hasFreshPt() ? lastPtMs : 0;
  const bool settled = ptPsiSettled();
  bbLox.update(arming.isArmed(), settled, pressureSampleMs);
  bbFuel.update(arming.isArmed(), settled, pressureSampleMs);

  // 6. Telemetry. Schedules advance even if a frame is dropped; retrying
  //    immediately would recreate the TX starvation the reserve guards against.
  static uint32_t lastTelemetryMs = 0;
  static uint32_t lastHeartbeatMs = 0;
  static uint32_t lastBbDebugMs = 0;
  static uint32_t lastPowerMs = 0;
  const uint32_t now = millis();

  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }
  if (now - lastHeartbeatMs >= BB_HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    sendBbHeartbeat(now);
  }
  if (now - lastBbDebugMs >= BB_DEBUG_INTERVAL_MS) {
    lastBbDebugMs = now;
    sendBbDebug();
  }
  if (now - lastPowerMs >= POWER_INTERVAL_MS) {
    lastPowerMs = now;
    sendPowerTelemetry();
  }

  // 7. Sequence completion
  checkSequenceComplete();

  // Rate cap: plenty for the scanners' 500 µs mux settle while avoiding a
  // 600 kHz spin that hammers ISRs and the power rail.
  delayMicroseconds(100);
}
