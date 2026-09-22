// DC Channel Hardware Test — PandaV2
// Menu-driven serial console for validating ADC + mux + actuation chain.

#include <Arduino.h>
#include <SPI.h>
#include "pins.h"
#include "BoardConfig.h"
#include <MCP23S17.h>
#include <MCP3561RT.h>

// ── Configuration ───────────────────────────────────────────────────

static constexpr uint8_t MUX_A_PINS[4] = {PIN_MUX_A_S0, PIN_MUX_A_S1, PIN_MUX_A_S2, PIN_MUX_A_S3};
static constexpr uint8_t MUX_B_PINS[4] = {PIN_MUX_B_S0, PIN_MUX_B_S1, PIN_MUX_B_S2, PIN_MUX_B_S3};
static constexpr uint8_t MUX_C_PINS[4] = {PIN_MUX_C_S0, PIN_MUX_C_S1, PIN_MUX_C_S2, PIN_MUX_C_S3};

static constexpr uint32_t CONV_TIMEOUT_US = 50000;
static constexpr uint16_t DEFAULT_NOISE_SAMPLES = 200;
static constexpr uint32_t DEFAULT_PULSE_MS = 500;

// ── Hardware instances ──────────────────────────────────────────────

MCP3561RT adc1(PIN_ADC1_CS, PIN_ADC1_IRQ, SPI,  SPI_ADC_SETTINGS, ADC1_VREF_V);
MCP3561RT adc2(PIN_ADC2_CS, PIN_ADC2_IRQ, SPI1, SPI_BUS1_SETTINGS, ADC2_VREF_V);
MCP23S17 ioexp(PIN_IOEXP_CS, SPI1, SPI_BUS1_SETTINGS, IOEXP_HW_ADDR);

// Same chip, same /CS, probed on SPI0 instead. docs/pinout.md marks the SPI1
// MOSI/SCK/MISO assignments as *inferred*, so this checks whether U35 (and by
// extension ADC2) actually hang off bus 0 with ADC1.
MCP23S17 ioexpBus0(PIN_IOEXP_CS, SPI, SPI_BUS1_SETTINGS, IOEXP_HW_ADDR);
MCP3561RT adc2Bus0(PIN_ADC2_CS, PIN_ADC2_IRQ, SPI, SPI_ADC_SETTINGS, ADC2_VREF_V);

// ── Mux helpers ─────────────────────────────────────────────────────

static void initMuxPins(const uint8_t pins[4]) {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
    }
}

static void setMuxChannel(const uint8_t pins[4], uint8_t ch) {
    for (uint8_t i = 0; i < 4; i++) {
        digitalWrite(pins[i], (ch >> i) & 1);
    }
}

// ── ADC read helpers ────────────────────────────────────────────────

struct ReadResult {
    bool ok;
    int32_t raw;
    float voltage;
};

static ReadResult singleRead(MCP3561RT& adc) {
    ReadResult r = {false, 0, 0.0f};

    adc.trigger();

    elapsedMicros timeout;
    while (!adc.dataReady()) {
        if (timeout > CONV_TIMEOUT_US) return r;
    }

    if (adc.readRaw(r.raw)) {
        r.voltage = adc.vref() * (float(r.raw) / 8388608.0f);
        r.ok = true;
    }
    return r;
}

static ReadResult readMuxChannel(MCP3561RT& adc, const uint8_t muxPins[4],
                                  MCP3561RT::Mux adcInput, uint8_t ch) {
    adc.setMux(adcInput, MCP3561RT::Mux::AGND);
    setMuxChannel(muxPins, ch);
    delayMicroseconds(T_MUX_SETTLE_US);
    return singleRead(adc);
}

// ── Serial input helpers ────────────────────────────────────────────

static int readSerialInt() {
    while (!Serial.available()) { yield(); }
    return Serial.parseInt();
}

// ── Test functions ──────────────────────────────────────────────────

static void testRegisterDump() {
    Serial.println("=== ADC Register Dump ===");

    const char* regNames[] = {
        "ADCDATA", "CONFIG0", "CONFIG1", "CONFIG2", "CONFIG3", "IRQ", "MUX"
    };

    for (uint8_t adcIdx = 0; adcIdx < 2; adcIdx++) {
        MCP3561RT& adc = (adcIdx == 0) ? adc1 : adc2;
        Serial.printf("\nADC%u:\n", adcIdx + 1);
        for (uint8_t reg = 0; reg <= 6; reg++) {
            uint8_t val = adc.readRegister(reg);
            Serial.printf("  [0x%02X] %-8s = 0x%02X (0b", reg, regNames[reg], val);
            for (int8_t b = 7; b >= 0; b--) Serial.print((val >> b) & 1);
            Serial.println(")");
        }
    }
    Serial.println();
}

static void testSingleChannel() {
    Serial.println("=== Single Channel Read ===");
    Serial.println("Select bank: 1=MuxA(ADC1) 2=MuxB(ADC1) 3=MuxC(ADC2)");

    int bank = readSerialInt();
    if (bank < 1 || bank > 3) { Serial.println("Invalid bank."); return; }

    Serial.printf("Channel (0-%u): ", NUM_MUX_A_CH - 1);
    int ch = readSerialInt();
    if (ch < 0 || ch > 15) { Serial.println("Invalid channel."); return; }

    MCP3561RT& adc       = (bank <= 2) ? adc1 : adc2;
    const uint8_t* pins  = (bank == 1) ? MUX_A_PINS : (bank == 2) ? MUX_B_PINS : MUX_C_PINS;
    MCP3561RT::Mux input = (bank == 2) ? MCP3561RT::Mux::CH1 : MCP3561RT::Mux::CH0;

    ReadResult r = readMuxChannel(adc, pins, input, (uint8_t)ch);

    if (r.ok) {
        Serial.printf("Bank %d CH%02d: raw=%d  voltage=%.6f V\n", bank, ch, r.raw, r.voltage);
    } else {
        Serial.printf("Bank %d CH%02d: TIMEOUT / READ FAIL\n", bank, ch);
    }
    Serial.println();
}

static void testSweep() {
    Serial.println("=== 16-Channel Sweep ===");
    Serial.println("Select bank: 1=MuxA(ADC1) 2=MuxB(ADC1) 3=MuxC(ADC2)");

    int bank = readSerialInt();
    if (bank < 1 || bank > 3) { Serial.println("Invalid bank."); return; }

    MCP3561RT& adc       = (bank <= 2) ? adc1 : adc2;
    const uint8_t* pins  = (bank == 1) ? MUX_A_PINS : (bank == 2) ? MUX_B_PINS : MUX_C_PINS;
    MCP3561RT::Mux input = (bank == 2) ? MCP3561RT::Mux::CH1 : MCP3561RT::Mux::CH0;

    Serial.printf("Bank %d sweep:\n", bank);
    Serial.println("  CH   |     Raw     |   Voltage (V)");
    Serial.println("  -----+-------------+--------------");

    for (uint8_t ch = 0; ch < 16; ch++) {
        ReadResult r = readMuxChannel(adc, pins, input, ch);
        if (r.ok) {
            Serial.printf("  %02d   | %10d  |  %+.6f\n", ch, r.raw, r.voltage);
        } else {
            Serial.printf("  %02d   |    FAIL     |     ---\n", ch);
        }
    }
    Serial.println();
}

static void testNoise() {
    Serial.println("=== Noise Statistics ===");
    Serial.println("Select bank: 1=MuxA(ADC1) 2=MuxB(ADC1) 3=MuxC(ADC2)");

    int bank = readSerialInt();
    if (bank < 1 || bank > 3) { Serial.println("Invalid bank."); return; }

    Serial.printf("Channel (0-%u): ", NUM_MUX_A_CH - 1);
    int ch = readSerialInt();
    if (ch < 0 || ch > 15) { Serial.println("Invalid channel."); return; }

    MCP3561RT& adc       = (bank <= 2) ? adc1 : adc2;
    const uint8_t* pins  = (bank == 1) ? MUX_A_PINS : (bank == 2) ? MUX_B_PINS : MUX_C_PINS;
    MCP3561RT::Mux input = (bank == 2) ? MCP3561RT::Mux::CH1 : MCP3561RT::Mux::CH0;

    adc.setMux(input, MCP3561RT::Mux::AGND);
    setMuxChannel(pins, (uint8_t)ch);
    delayMicroseconds(T_MUX_SETTLE_US);

    Serial.printf("Sampling %u readings on bank %d CH%02d...\n", DEFAULT_NOISE_SAMPLES, bank, ch);

    double sum = 0.0, sumSq = 0.0;
    int32_t minRaw = INT32_MAX, maxRaw = INT32_MIN;
    uint16_t good = 0;

    for (uint16_t i = 0; i < DEFAULT_NOISE_SAMPLES; i++) {
        ReadResult r = singleRead(adc);
        if (!r.ok) continue;

        good++;
        sum += r.raw;
        sumSq += (double)r.raw * (double)r.raw;
        if (r.raw < minRaw) minRaw = r.raw;
        if (r.raw > maxRaw) maxRaw = r.raw;
    }

    if (good < 2) {
        Serial.println("Too few valid samples.");
        return;
    }

    double mean = sum / good;
    double variance = (sumSq / good) - (mean * mean);
    double stddev = sqrt(variance);
    double meanV = adc.vref() * (mean / 8388608.0);
    double stddevV = adc.vref() * (stddev / 8388608.0);

    Serial.printf("  Samples:  %u / %u\n", good, DEFAULT_NOISE_SAMPLES);
    Serial.printf("  Mean:     %.1f counts  (%.6f V)\n", mean, meanV);
    Serial.printf("  Std Dev:  %.1f counts  (%.6f V)\n", stddev, stddevV);
    Serial.printf("  Peak-Pk:  %d counts  (min=%d, max=%d)\n",
                  maxRaw - minRaw, minRaw, maxRaw);
    Serial.printf("  ENOB:     %.1f bits\n", log2(8388608.0 / stddev));
    Serial.println();
}

// ── Arming helpers ──────────────────────────────────────────────────

static bool armed = false;

static void setArmed(bool state) {
    armed = state;
    digitalWrite(PIN_ARM, state ? HIGH : LOW);
}

static void testArming() {
    Serial.println("=== Arm / Disarm ===");
    Serial.printf("Current state: %s\n", armed ? "ARMED" : "DISARMED");
    Serial.println("  1  Arm   (PIN_ARM high)");
    Serial.println("  0  Disarm (PIN_ARM low)");
    Serial.print("> ");

    int sub = readSerialInt();
    if (sub != 0 && sub != 1) { Serial.println("Invalid option."); return; }

    setArmed(sub == 1);
    Serial.printf("Board is now %s (pin %u = %s)\n",
                  armed ? "ARMED" : "DISARMED", PIN_ARM, armed ? "HIGH" : "LOW");
    Serial.println();
}

// ── Actuation helpers ───────────────────────────────────────────────

static void dcSetChannel(uint8_t ch, bool state) {
    // ch is 1-indexed → ACTUATEch on the expander
    ioexp.setChannel(ch, state);
}

static void dcAllOff() {
    ioexp.allOff();
}

// ── Expander diagnostics ────────────────────────────────────────────

static void printByteBin(uint8_t v) {
    for (int8_t b = 7; b >= 0; b--) Serial.print((v >> b) & 1);
}

// Write/read a known pattern to DEFVALA, which drives no pins. Pure link test.
static bool scratchTest(MCP23S17& dev, const char* label) {
    const uint8_t patterns[] = {0xA5, 0x5A, 0xFF};
    bool ok = true;
    for (uint8_t p : patterns) {
        dev.writeRegister(MCP23S17::REG_DEFVALA, p);
        uint8_t got = dev.readRegister(MCP23S17::REG_DEFVALA);
        Serial.printf("  %s: wrote 0x%02X, read 0x%02X %s\n", label, p, got,
                      got == p ? "OK" : "<-- MISMATCH");
        if (got != p) ok = false;
    }
    dev.writeRegister(MCP23S17::REG_DEFVALA, 0x00);
    return ok;
}

// Both SPI1 devices are dead but ADC1 on SPI0 works. The SPI1 MOSI/SCK/MISO
// assignments are only *inferred* in docs/pinout.md, so retry both devices on
// SPI0 using their confirmed /CS pins.
static void testProbeBus0() {
    Serial.println("=== Probe ADC2 + Expander on SPI0 ===");
    Serial.println("SPI0 = MOSI 11 / MISO 12 / SCK 13, CS unchanged");
    Serial.printf("  expander CS=%u, adc2 CS=%u\n\n", PIN_IOEXP_CS, PIN_ADC2_CS);

    ioexpBus0.begin();
    bool expOk = scratchTest(ioexpBus0, "U35 ");

    bool adcOk = adc2Bus0.begin();
    Serial.printf("\n  ADC2 on SPI0 begin(): %s\n", adcOk ? "OK" : "FAIL");
    uint8_t nonZero = 0;
    for (uint8_t reg = 1; reg <= 6; reg++) {
        uint8_t v = adc2Bus0.readRegister(reg);
        Serial.printf("    [0x%02X] = 0x%02X\n", reg, v);
        nonZero |= v;
    }

    Serial.println();
    if (expOk || nonZero) {
        Serial.println("  ** Device(s) RESPOND on SPI0. The board wires ADC2/U35");
        Serial.println("     to bus 0, not the inferred LPSPI3 pins 26/27/39.");
        Serial.println("     Update pins.h + main.cpp to use SPI for both.");
    } else {
        Serial.println("  No response on SPI0 either. Check U35/U38 power and");
        Serial.println("  ring out MOSI/SCK/MISO from the Teensy to each chip.");
    }
    Serial.println();
}

static void testExpanderDiag() {
    Serial.println("=== MCP23S17 Diagnostics ===");
    Serial.printf("CS pin %u, bus SPI1 (MOSI 26 / SCK 27 / MISO 39)\n\n",
                  PIN_IOEXP_CS);

    struct { const char* name; uint8_t addr; } regs[] = {
        {"IOCON",  MCP23S17::REG_IOCON},
        {"IODIRA", MCP23S17::REG_IODIRA},
        {"IODIRB", MCP23S17::REG_IODIRB},
        {"OLATA",  MCP23S17::REG_OLATA},
        {"OLATB",  MCP23S17::REG_OLATB},
        {"GPIOA",  MCP23S17::REG_GPIOA},
        {"GPIOB",  MCP23S17::REG_GPIOB},
    };

    uint8_t allOr = 0x00, allAnd = 0xFF;
    for (auto& r : regs) {
        uint8_t v = ioexp.readRegister(r.addr);
        allOr |= v;
        allAnd &= v;
        Serial.printf("  [0x%02X] %-6s = 0x%02X (0b", r.addr, r.name, v);
        printByteBin(v);
        Serial.println(")");
    }

    Serial.println();

    // A chip that isn't responding reads as all-zero (MISO idle low / not
    // connected) or all-ones (MISO floating high). Real silicon with the
    // driver's init returns IODIRA/B = 0x00 but IOCON = 0x08.
    if (allOr == 0x00) {
        Serial.println("  ** All registers read 0x00 — no response on MISO.");
        Serial.println("     Check: CS pin/net, MISO routing (pin 39), RESET (U35 pin 9)");
    } else if (allAnd == 0xFF) {
        Serial.println("  ** All registers read 0xFF — MISO floating.");
        Serial.println("     Check: MISO routing (pin 39), device power");
    }

    // Scratch-register test. DEFVALA drives nothing, so this probes the SPI
    // link alone without touching any output. Every other register above
    // legitimately reads back 0x00 (that IS what begin() wrote), so only a
    // register we can set to a known non-zero value proves anything.
    Serial.println("Scratch register test (DEFVALA, no effect on outputs):");
    bool linkOk = scratchTest(ioexp, "U35");

    if (!linkOk) {
        Serial.println("  ** SPI link to U35 is dead. Since ADC2 shares MOSI/");
        Serial.println("     SCK/MISO on this bus, run menu option 1: if ADC2's");
        Serial.println("     registers read back sane values, the shared bus is");
        Serial.println("     fine and the fault is specific to U35 -- /CS net on");
        Serial.println("     pin 37, /RESET (U35 pin 9), or U35 VDD.");
        Serial.println();
        return;
    }

    Serial.println("Running OLAT loopback (outputs pulse, then all off)...");
    uint16_t readback = 0;
    bool ok = ioexp.selfTest(readback);
    Serial.printf("  wrote 0xA55A, read 0x%04X -> %s\n", readback,
                  ok ? "PASS" : "FAIL");

    if (ok) {
        Serial.println("  SPI link and expander are healthy.");
        Serial.println("  If transistors still don't switch, the fault is");
        Serial.println("  downstream: ACTUATE net, Q8_x array, or drive supply.");
    } else {
        Serial.println("  SPI write/read path is broken — fix this before");
        Serial.println("  suspecting the transistor array.");
    }
    Serial.println();
}

// ── Actuation test functions ────────────────────────────────────────

static void testActuateSingle() {
    Serial.println("=== Single Actuator Toggle ===");
    Serial.printf("Channel (1-%u): ", NUM_ACTUATORS);
    int ch = readSerialInt();
    if (ch < 1 || ch > (int)NUM_ACTUATORS) { Serial.println("Invalid channel."); return; }

    Serial.printf("Pulse duration ms (%lu default): ", DEFAULT_PULSE_MS);
    int dur = readSerialInt();
    if (dur <= 0) dur = DEFAULT_PULSE_MS;

    Serial.printf("Actuating CH%02d for %d ms...", ch, dur);
    dcSetChannel((uint8_t)ch, true);
    delay(dur);
    dcSetChannel((uint8_t)ch, false);
    Serial.println(" done.");
    Serial.println();
}

static void testActuateWalk() {
    Serial.println("=== Walk All Actuators ===");
    Serial.printf("Pulse duration ms per channel (%lu default): ", DEFAULT_PULSE_MS);
    int dur = readSerialInt();
    if (dur <= 0) dur = DEFAULT_PULSE_MS;

    for (uint8_t ch = 1; ch <= NUM_ACTUATORS; ch++) {
        Serial.printf("  CH%02d (ACTUATE%u) ON...", ch, ch);
        dcSetChannel(ch, true);
        delay(dur);
        dcSetChannel(ch, false);
        Serial.println(" OFF");
        delay(50);
    }
    Serial.println("Walk complete.");
    Serial.println();
}

static void testActuateAllOnOff() {
    Serial.println("=== All Actuators On/Off ===");
    Serial.printf("Pulse duration ms (%lu default): ", DEFAULT_PULSE_MS);
    int dur = readSerialInt();
    if (dur <= 0) dur = DEFAULT_PULSE_MS;

    Serial.printf("All ON for %d ms...", dur);
    for (uint8_t i = 1; i <= NUM_ACTUATORS; i++)
        ioexp.setChannel(i, true);
    delay(dur);
    dcAllOff();
    Serial.println(" All OFF.");
    Serial.println();
}

// Latched: stays in the new state after returning to the menu, so channels can
// be probed with a meter while energized. Any channel on -> all off.
static void testActuateToggleAll() {
    Serial.println("=== Toggle All Actuators ===");
    if (ioexp.getState()) {
        dcAllOff();
        Serial.println("All channels OFF.");
    } else {
        ioexp.setAll(0xFFFF);
        Serial.println("All channels ON (latched) - run this again to turn them off.");
        if (!armed)
            Serial.println("Note: board is DISARMED, so outputs won't energize (menu 6).");
    }
    Serial.println();
}

static void testActuation() {
    Serial.println("=== Actuation Test ===");
    Serial.println("  1  Toggle single channel");
    Serial.println("  2  Walk all channels");
    Serial.println("  3  All on / all off");
    Serial.println("  4  Toggle all channels (latched)");
    Serial.print("> ");

    int sub = readSerialInt();
    switch (sub) {
        case 1: testActuateSingle();   break;
        case 2: testActuateWalk();     break;
        case 3: testActuateAllOnOff(); break;
        case 4: testActuateToggleAll(); break;
        default: Serial.println("Invalid option."); break;
    }
}

// ── Current-sense check ─────────────────────────────────────────────
// Localizes a flat current reading. First checks that ADC2 itself measures
// known inputs, then
// reads mux C channel N-1 with ACTUATE(N) off and on. INA181A1 x20 into
// 100 mΩ: expect ~2 V per amp while energized.

static void printAdc2Input(const char* label, MCP3561RT::Mux vinp) {
    adc2.setMux(vinp, MCP3561RT::Mux::AGND);
    delayMicroseconds(T_MUX_SETTLE_US);
    ReadResult r = singleRead(adc2);
    if (r.ok)
        Serial.printf("  %-22s raw=%9d  %+.6f V\n", label, r.raw, r.voltage);
    else
        Serial.printf("  %-22s TIMEOUT\n", label);
}

static void testCurrentSense() {
    Serial.println("=== Current-Sense Check (Mux C -> ADC2) ===");
    Serial.printf("Actuator channel (1-%u): ", NUM_ACTUATORS);
    int ch = readSerialInt();
    if (ch < 1 || ch > (int)NUM_ACTUATORS) { Serial.println("Invalid channel."); return; }
    const uint8_t muxCh = currentMuxCh((uint8_t)ch);

    Serial.println("ADC2 self-check (all vs AGND):");
    printAdc2Input("AGND   (expect ~0)", MCP3561RT::Mux::AGND);
    printAdc2Input("REFIN+ (expect ~FS)", MCP3561RT::Mux::REFIN_P);
    printAdc2Input("AVDD   (clips at FS)", MCP3561RT::Mux::AVDD);
    setMuxChannel(MUX_C_PINS, muxCh);
    printAdc2Input("CH0 (mux C COM)", MCP3561RT::Mux::CH0);

    ReadResult off = readMuxChannel(adc2, MUX_C_PINS, MCP3561RT::Mux::CH0, muxCh);

    const bool wasArmed = armed;
    setArmed(true);
    dcSetChannel((uint8_t)ch, true);
    delay(DEFAULT_PULSE_MS);  // let the solenoid current reach steady state
    ReadResult on = readMuxChannel(adc2, MUX_C_PINS, MCP3561RT::Mux::CH0, muxCh);
    const uint16_t gpio = ioexp.readGpio();
    dcSetChannel((uint8_t)ch, false);
    setArmed(wasArmed);

    Serial.printf("Mux C CH%02d / ACTUATE%d:\n", muxCh, ch);
    if (off.ok)
        Serial.printf("  OFF  %+.6f V  -> %.4f A\n", off.voltage, off.voltage * 0.5f);
    else
        Serial.println("  OFF  TIMEOUT");
    if (on.ok)
        Serial.printf("  ON   %+.6f V  -> %.4f A\n", on.voltage, on.voltage * 0.5f);
    else
        Serial.println("  ON   TIMEOUT");
    // readGpio(): bit15 = ACTUATE1 ... bit0 = ACTUATE16
    Serial.printf("  Expander pin for ACTUATE%d read back %s while ON\n", ch,
                  (gpio >> (NUM_ACTUATORS - ch)) & 1 ? "HIGH" : "LOW (did not drive!)");
    Serial.println();
}

// ── Menu ────────────────────────────────────────────────────────────

static void printMenu() {
    Serial.println("──────────────────────────────────");
    Serial.println("  PandaV2 DC Channel Test");
    Serial.println("──────────────────────────────────");
    Serial.println("  1  ADC register dump");
    Serial.println("  2  Single channel read");
    Serial.println("  3  16-channel sweep");
    Serial.println("  4  Noise statistics");
    Serial.println("  5  Actuation test");
    Serial.println("  6  Arm / disarm board");
    Serial.println("  7  Expander diagnostics");
    Serial.println("  8  Probe ADC2 + expander on SPI0");
    Serial.println("  9  Current-sense check (ADC2 + one channel)");
    Serial.println("──────────────────────────────────");
    Serial.print("> ");
}

// ── Entry points ────────────────────────────────────────────────────

void setup() {
    Serial.begin(DEBUG_BAUD);
    while (!Serial && millis() < 3000) {}

    // Park every chip select HIGH before any bus traffic — see src/main.cpp.
    // Two devices share SPI1, so an un-driven /CS on one corrupts the other's
    // init transactions.
    for (uint8_t cs : {PIN_ADC1_CS, PIN_ADC2_CS, PIN_IOEXP_CS}) {
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);
    }

    SPI.begin();
    SPI1.setMISO(PIN_SPI1_MISO);  // default is pin 1; hardware uses pin 39
    SPI1.begin();

    initMuxPins(MUX_A_PINS);
    initMuxPins(MUX_B_PINS);
    initMuxPins(MUX_C_PINS);

    // DC solenoid outputs via the MCP23S17 — brought up before the ADCs so its
    // /CS is driven high and can't squat on the shared SPI1 bus.
    ioexp.begin();

    // Arm line — held low (disarmed) until explicitly toggled
    pinMode(PIN_ARM, OUTPUT);
    digitalWrite(PIN_ARM, LOW);

    bool adc1_ok = adc1.begin();
    bool adc2_ok = adc2.begin();

    Serial.println("\nPandaV2 DC Channel Test");
    Serial.printf("ADC1:  %s\n", adc1_ok ? "OK" : "FAIL");
    Serial.printf("ADC2:  %s\n", adc2_ok ? "OK" : "FAIL");
    Serial.printf("IOEXP: %u actuator channels via MCP23S17 (CS=%u)\n",
                  NUM_ACTUATORS, PIN_IOEXP_CS);
    Serial.printf("Arm:   DISARMED (pin %u low)\n", PIN_ARM);

    printMenu();
}

void loop() {
    if (!Serial.available()) return;

    int cmd = Serial.parseInt();

    switch (cmd) {
        case 1: testRegisterDump();  break;
        case 2: testSingleChannel(); break;
        case 3: testSweep();         break;
        case 4: testNoise();         break;
        case 5: testActuation();     break;
        case 6: testArming();        break;
        case 7: testExpanderDiag();  break;
        case 8: testProbeBus0();     break;
        case 9: testCurrentSense();  break;
        default: break;
    }

    printMenu();
}
