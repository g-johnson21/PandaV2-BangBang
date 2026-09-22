// Sensor Hardware Test — PandaV2
// Interactive test for PTs (mux A) and DC channel current sense (mux C).
// Mux B (load cells + thermocouples) is unassembled and not scanned.
// Run via USB serial. Select a test from the menu.

#include <Arduino.h>
#include <SPI.h>
#include "pins.h"
#include "BoardConfig.h"
#include <MCP3561RT.h>
#include "SensorConfig.h"

// ── Hardware ────────────────────────────────────────────────────────

MCP3561RT adc1(PIN_ADC1_CS, PIN_ADC1_IRQ, SPI,  SPI_ADC_SETTINGS, ADC1_VREF_V);
MCP3561RT adc2(PIN_ADC2_CS, PIN_ADC2_IRQ, SPI1, SPI_BUS1_SETTINGS, ADC2_VREF_V);

static constexpr uint8_t MUX_A_PINS[4] = {PIN_MUX_A_S0, PIN_MUX_A_S1, PIN_MUX_A_S2, PIN_MUX_A_S3};
static constexpr uint8_t MUX_B_PINS[4] = {PIN_MUX_B_S0, PIN_MUX_B_S1, PIN_MUX_B_S2, PIN_MUX_B_S3};
static constexpr uint8_t MUX_C_PINS[4] = {PIN_MUX_C_S0, PIN_MUX_C_S1, PIN_MUX_C_S2, PIN_MUX_C_S3};

static constexpr uint32_t CONV_TIMEOUT_US = 50000;

static bool adc1_ok = false;
static bool adc2_ok = false;

// ── Helpers ─────────────────────────────────────────────────────────

static void initMuxPins(const uint8_t pins[4]) {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
    }
}

static void setMuxChannel(const uint8_t pins[4], uint8_t ch) {
    for (uint8_t i = 0; i < 4; i++)
        digitalWrite(pins[i], (ch >> i) & 1);
}

struct ReadResult { bool ok; int32_t raw; float voltage; };

// Scales by the ADC's own reference (ADCn_VREF_V via the constructor).
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

static ReadResult readMuxCh(MCP3561RT& adc, const uint8_t muxPins[4],
                             MCP3561RT::Mux adcInput, uint8_t ch) {
    adc.setMux(adcInput, MCP3561RT::Mux::AGND);
    setMuxChannel(muxPins, ch);
    delayMicroseconds(T_MUX_SETTLE_US);
    return singleRead(adc);
}

static float readBoardTemp(MCP3561RT& adc) {
    adc.setMux(MCP3561RT::Mux::TEMP_P, MCP3561RT::Mux::TEMP_N);
    delayMicroseconds(T_MUX_SETTLE_US);
    ReadResult r = singleRead(adc);
    if (!r.ok) return -999.0f;
    return (r.voltage - 0.492f) / 0.00176f;
}

static int readSerialInt() {
    while (!Serial.available()) { yield(); }
    return Serial.parseInt();
}

// ── 1. PT sweep ────────────────────────────────────────────────────

static void testPTSweep() {
    Serial.println("=== PT Sweep (Mux A → ADC1 CH0) ===");
    if (!adc1_ok) { Serial.println("ADC1 not initialized."); return; }

    Serial.println("  CH  |     Raw     |   Voltage (V)  |  Converted");
    Serial.println("  ----+-------------+----------------+-----------");

    for (uint8_t ch = 0; ch < NUM_PT_CH; ch++) {
        ReadResult r = readMuxCh(adc1, MUX_A_PINS, MCP3561RT::Mux::CH0, ch);
        if (r.ok) {
            float conv = convertPT(r.voltage, ch);
            Serial.printf("  %02d  | %10d  |  %+.6f     |  %.4f\n",
                          ch, r.raw, r.voltage, conv);
        } else {
            Serial.printf("  %02d  |    FAIL     |     ---        |   ---\n", ch);
        }
    }
    Serial.println();
}

// ── 2. DC channel current sweep ────────────────────────────────────

static void testCurrentSweep() {
    Serial.println("=== DC Current Sweep (Mux C → ADC2 CH0) ===");
    Serial.println("ACTUATE n is sensed on mux C ch 16-n (INA181A1, x20).");
    if (!adc2_ok) { Serial.println("ADC2 not initialized."); return; }

    Serial.println("  ACTUATE  |  CH  |     Raw     |   Voltage (V)  |  Current (A)");
    Serial.println("  ---------+------+-------------+----------------+-------------");

    for (uint8_t act = 1; act <= NUM_CURRENT_CH; act++) {
        const uint8_t ch = currentMuxCh(act);
        ReadResult r = readMuxCh(adc2, MUX_C_PINS, MCP3561RT::Mux::CH0, ch);
        if (r.ok) {
            float amps = convertCurrent(r.voltage);
            Serial.printf("     %2u    |  %02u  | %10d  |  %+.6f     |  %.4f%s\n",
                          act, ch, r.raw, r.voltage, amps,
                          amps >= CURRENT_FULL_SCALE_A * 0.99f ? "  <-- CLIPPED" : "");
        } else {
            Serial.printf("     %2u    |  %02u  |    FAIL     |     ---        |   ---\n",
                          act, ch);
        }
    }
    Serial.println();
}

// ── 3. Full sweep (all sensors) ────────────────────────────────────

static void testFullSweep() {
    testPTSweep();
    testCurrentSweep();
}

// ── 4. Continuous stream ───────────────────────────────────────────

static void testStream() {
    Serial.println("=== Continuous Stream ===");
    Serial.println("Streaming all sensors. Send any character to stop.");
    Serial.println("Format: PT0..15 (mA), CUR0..15 (A, = ACTUATE 1..16)");
    Serial.println();

    while (true) {
        if (Serial.available()) {
            Serial.read();
            break;
        }

        // PTs
        if (adc1_ok) {
            Serial.print("PT:");
            for (uint8_t ch = 0; ch < NUM_PT_CH; ch++) {
                ReadResult r = readMuxCh(adc1, MUX_A_PINS, MCP3561RT::Mux::CH0, ch);
                if (ch > 0) Serial.print(',');
                if (r.ok)
                    Serial.printf("%.5f", convertPT(r.voltage, ch));
                else
                    Serial.print("NaN");
            }
            Serial.println();
        }

        // DC channel current sense
        if (adc2_ok) {
            Serial.print("CUR:");
            for (uint8_t act = 1; act <= NUM_CURRENT_CH; act++) {
                ReadResult r = readMuxCh(adc2, MUX_C_PINS, MCP3561RT::Mux::CH0,
                                         currentMuxCh(act));
                if (act > 1) Serial.print(',');
                if (r.ok)
                    Serial.printf("%.4f", convertCurrent(r.voltage));
                else
                    Serial.print("NaN");
            }
            Serial.println();
        }

        Serial.println("---");
        delay(100);
    }

    Serial.println("Stream stopped.");
    Serial.println();
}

// ── 5. Board temperature ───────────────────────────────────────────

static void testBoardTemp() {
    Serial.println("=== Board Temperature (ADC internal sensor) ===");

    if (adc1_ok) {
        float t1 = readBoardTemp(adc1);
        Serial.printf("  ADC1 internal temp: %.1f °C %s\n", t1, (t1 < -900.0f) ? "(FAIL)" : "");
    }
    if (adc2_ok) {
        float t2 = readBoardTemp(adc2);
        Serial.printf("  ADC2 internal temp: %.1f °C %s\n", t2, (t2 < -900.0f) ? "(FAIL)" : "");
    }
    Serial.println();
}

// ── 6. ADC register dump ───────────────────────────────────────────

static void testRegDump() {
    const char* regNames[] = {"ADCDATA","CONFIG0","CONFIG1","CONFIG2","CONFIG3","IRQ","MUX"};
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

// ── Menu ────────────────────────────────────────────────────────────

static void printMenu() {
    Serial.println("──────────────────────────────────");
    Serial.println("  PandaV2 Sensor Test");
    Serial.println("──────────────────────────────────");
    Serial.println("  1  PT sweep (Mux A -> ADC1, 16ch)");
    Serial.println("  2  DC current sweep (Mux C -> ADC2, 16ch)");
    Serial.println("  3  Full sweep (all sensors)");
    Serial.println("  4  Continuous stream");
    Serial.println("  5  Board temperature (ADC internal sensor)");
    Serial.println("  6  ADC register dump");
    Serial.println("──────────────────────────────────");
    Serial.print("> ");
}

// ── Entry points ────────────────────────────────────────────────────

void setup() {
    Serial.begin(DEBUG_BAUD);
    while (!Serial && millis() < 3000) {}

    SPI.begin();
    SPI1.setMISO(PIN_SPI1_MISO);
    SPI1.begin();

    initMuxPins(MUX_A_PINS);
    initMuxPins(MUX_B_PINS);
    initMuxPins(MUX_C_PINS);

    adc1_ok = adc1.begin();
    adc2_ok = adc2.begin();

    Serial.println("\nPandaV2 Sensor Test");
    Serial.printf("ADC1: %s\n", adc1_ok ? "OK" : "FAIL");
    Serial.printf("ADC2: %s\n", adc2_ok ? "OK" : "FAIL");

    if (adc1_ok) {
        float t = readBoardTemp(adc1);
        Serial.printf("Board temp (ADC1): %.1f °C\n", t);
    }

    printMenu();
}

void loop() {
    if (!Serial.available()) return;
    int cmd = Serial.parseInt();

    switch (cmd) {
        case 1: testPTSweep();      break;
        case 2: testCurrentSweep(); break;
        case 3: testFullSweep();    break;
        case 4: testStream();       break;
        case 5: testBoardTemp();    break;
        case 6: testRegDump();      break;
        default: break;
    }

    printMenu();
}
