#pragma once
#include <Arduino.h>
#include <MCP3561RT.h>
#include "pins.h"
#include "BoardConfig.h"

// Generic mux-scanning ADC reader.
// Each Scanner instance owns one ADC and one or more mux banks.
// The state machine is non-blocking: call update() from the main loop.
//
// Priority ("fast") slots: an optional list of (bank, channel) pairs that are
// re-sampled between every ordinary channel. The bang-bang PTs use this so
// they refresh at a few hundred Hz instead of once per full 32-channel sweep.
// The ordinary round-robin skips fast slots so they are never read twice.

struct MuxBank {
    const uint8_t pins[4];      // S0..S3 GPIO
    uint8_t numChannels;
    MCP3561RT::Mux adcInput;    // which ADC input the mux feeds
    float* out;                 // output buffer (caller-owned)
    // No vref here: conversions use the owning ADC's reference (adc.vref()),
    // so a bank can never disagree with the chip it's actually read through.
};

struct ScanSlot {
    uint8_t bank;
    uint8_t ch;
};

class Scanner {
public:
    Scanner(MCP3561RT& adc, MuxBank* banks, uint8_t numBanks,
            const ScanSlot* fast = nullptr, uint8_t numFast = 0);

    void begin();
    void update();

    bool scanComplete() const { return _scanComplete; }
    void clearScanComplete() { _scanComplete = false; }

    // Incremented each time every fast slot has been converted successfully
    // in one pass. A pass containing a failed/timed-out read does not count,
    // so a stalled ADC shows up as this counter freezing.
    uint32_t fastSweepCount() const { return _fastSweeps; }

    // Conversions abandoned after ADC_CONV_TIMEOUT_US.
    uint32_t convTimeouts() const { return _timeouts; }

    // True between conversions — safe to borrow the ADC (readBoardTemp).
    bool isIdle() const { return _state == IDLE; }

    // Read ADC internal temperature sensor (for TC cold junction compensation).
    // Blocking — only call during setup or when isIdle().
    float readBoardTemp();

private:
    enum State : uint8_t { IDLE, WAIT_MUX, WAIT_CONV };

    MCP3561RT& _adc;
    MuxBank* _banks;
    uint8_t _numBanks;
    const ScanSlot* _fast;
    uint8_t _numFast;

    State _state = IDLE;
    uint8_t _bankIdx = 0;       // slow round-robin position
    uint8_t _chanIdx = 0;
    uint8_t _fastIdx = 0;
    bool _inFast = false;       // current slot comes from the fast list
    bool _hasSlow = true;       // false if every channel is a fast slot
    bool _fastPassOk = true;
    uint32_t _fastSweeps = 0;
    uint32_t _timeouts = 0;
    uint8_t _curBank = 0;       // slot being converted
    uint8_t _curChan = 0;
    elapsedMicros _timer;
    bool _scanComplete = false;

    void selectMuxChannel(const MuxBank& bank, uint8_t ch);
    bool isFastSlot(uint8_t bank, uint8_t ch) const;
    void advanceSlow();
    void finishSlot(bool ok);
};
