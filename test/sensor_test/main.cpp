// Sensor Hardware Test — PandaV2
// Interactive test for PTs, load cells, thermocouples, and current sense.
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

// ── 2. Load cell sweep ─────────────────────────────────────────────

static void testLCSweep() {
    Serial.println("=== Load Cell Sweep (Mux C ch 0-7 → ADC2 CH0) ===");
    if (!adc2_ok) { Serial.println("ADC2 not initialized."); return; }

    Serial.println("  CH  |     Raw     |   Voltage (V)  |  Converted");
    Serial.println("  ----+-------------+----------------+-----------");

    for (uint8_t i = 0; i < NUM_LC_CH; i++) {
        uint8_t muxCh = MUX_C_LC_START + i;
        ReadResult r = readMuxCh(adc2, MUX_C_PINS, MCP3561RT::Mux::CH0, muxCh);
        if (r.ok) {
            float conv = convertLC(r.voltage, i);
            Serial.printf("  %02d  | %10d  |  %+.6f     |  %.4f\n",
                          i, r.raw, r.voltage, conv);
        } else {
            Serial.printf("  %02d  |    FAIL     |     ---        |   ---\n", i);
        }
    }
    Serial.println();
}

// ── 3. Thermocouple sweep ──────────────────────────────────────────

static void testTCSweep() {
    Serial.println("=== Thermocouple Sweep (Mux C ch 8-15 → ADC2 CH0) ===");
    if (!adc2_ok) { Serial.println("ADC2 not initialized."); return; }

    float bTemp = readBoardTemp(adc2);
    Serial.printf("Board temp (CJC): %.1f °C\n", bTemp);
    if (bTemp < -900.0f) {
        Serial.println("WARNING: board temp read failed, using 25 °C");
        bTemp = 25.0f;
    }

    Serial.println("  CH  |     Raw     |   Voltage (V)  |  Temp (°C)");
    Serial.println("  ----+-------------+----------------+-----------");

    for (uint8_t i = 0; i < NUM_TC_CH; i++) {
        uint8_t muxCh = MUX_C_TC_START + i;
        ReadResult r = readMuxCh(adc2, MUX_C_PINS, MCP3561RT::Mux::CH0, muxCh);
        if (r.ok) {
            float temp = convertTC(r.voltage, i, bTemp);
            Serial.printf("  %02d  | %10d  |  %+.6f     |  %.1f\n",
                          i, r.raw, r.voltage, temp);
        } else {
            Serial.printf("  %02d  |    FAIL     |     ---        |   ---\n", i);
        }
    }
    Serial.println();
}

// ── 4. Current sense sweep ─────────────────────────────────────────

static void testCurrentSweep() {
    Serial.println("=== Current Sense Sweep (Mux B → ADC1 CH1) ===");
    if (!adc1_ok) { Serial.println("ADC1 not initialized."); return; }

    Serial.println("  CH  |     Raw     |   Voltage (V)  |  Current (A)");
    Serial.println("  ----+-------------+----------------+-------------");

    for (uint8_t ch = 0; ch < NUM_MUX_B_CH; ch++) {
        ReadResult r = readMuxCh(adc1, MUX_B_PINS, MCP3561RT::Mux::CH1, ch);
        if (r.ok) {
            float amps = convertCurrent(r.voltage);
            Serial.printf("  %02d  | %10d  |  %+.6f     |  %.4f\n",
                          ch, r.raw, r.voltage, amps);
        } else {
            Serial.printf("  %02d  |    FAIL     |     ---        |   ---\n", ch);
        }
    }
    Serial.println();
}

// ── 5. Full sweep (all sensors) ────────────────────────────────────

static void testFullSweep() {
    testPTSweep();
    testCurrentSweep();
    testLCSweep();
    testTCSweep();
}

// ── 6. Continuous stream ───────────────────────────────────────────

static void testStream() {
    Serial.println("=== Continuous Stream ===");
    Serial.println("Streaming all sensors. Send any character to stop.");
    Serial.println("Format: PT0..15, LC0..7, TC0..7, CUR0..15");
    Serial.println();

    float bTemp = 25.0f;
    uint32_t lastBoardTemp = 0;

    while (true) {
        if (Serial.available()) {
            Serial.read();
            break;
        }

        // Update board temp every 2s
        if (millis() - lastBoardTemp > 2000) {
            if (adc2_ok) {
                float t = readBoardTemp(adc2);
                if (t > -900.0f) bTemp = t;
            }
            lastBoardTemp = millis();
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

        // Load cells
        if (adc2_ok) {
            Serial.print("LC:");
            for (uint8_t i = 0; i < NUM_LC_CH; i++) {
                ReadResult r = readMuxCh(adc2, MUX_C_PINS, MCP3561RT::Mux::CH0, MUX_C_LC_START + i);
                if (i > 0) Serial.print(',');
                if (r.ok)
                    Serial.printf("%.5f", convertLC(r.voltage, i));
                else
                    Serial.print("NaN");
            }
            Serial.println();
        }

        // Thermocouples
        if (adc2_ok) {
            Serial.print("TC:");
            for (uint8_t i = 0; i < NUM_TC_CH; i++) {
                ReadResult r = readMuxCh(adc2, MUX_C_PINS, MCP3561RT::Mux::CH0, MUX_C_TC_START + i);
                if (i > 0) Serial.print(',');
                if (r.ok)
                    Serial.printf("%.1f", convertTC(r.voltage, i, bTemp));
                else
                    Serial.print("NaN");
            }
            Serial.println();
        }

        // Current sense
        if (adc1_ok) {
            Serial.print("CUR:");
            for (uint8_t ch = 0; ch < NUM_MUX_B_CH; ch++) {
                ReadResult r = readMuxCh(adc1, MUX_B_PINS, MCP3561RT::Mux::CH1, ch);
                if (ch > 0) Serial.print(',');
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

// ── 7. Board temperature ───────────────────────────────────────────

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

// ── 8. ADC register dump ───────────────────────────────────────────

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
    Serial.println("  1  PT sweep (Mux A, 16ch)");
    Serial.println("  2  Load cell sweep (Mux C, 8ch)");
    Serial.println("  3  Thermocouple sweep (Mux C, 8ch)");
    Serial.println("  4  Current sense sweep (Mux B, 16ch)");
    Serial.println("  5  Full sweep (all sensors)");
    Serial.println("  6  Continuous stream");
    Serial.println("  7  Board temperature");
    Serial.println("  8  ADC register dump");
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
        case 2: testLCSweep();      break;
        case 3: testTCSweep();      break;
        case 4: testCurrentSweep(); break;
        case 5: testFullSweep();    break;
        case 6: testStream();       break;
        case 7: testBoardTemp();    break;
        case 8: testRegDump();      break;
        default: break;
    }

    printMenu();
}
