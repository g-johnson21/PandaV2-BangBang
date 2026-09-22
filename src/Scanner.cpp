#include "Scanner.h"

Scanner::Scanner(MCP3561RT& adc, MuxBank* banks, uint8_t numBanks,
                 const ScanSlot* fast, uint8_t numFast)
    : _adc(adc), _banks(banks), _numBanks(numBanks),
      _fast(fast), _numFast(fast ? numFast : 0)
{
}

void Scanner::begin() {
    for (uint8_t b = 0; b < _numBanks; b++) {
        for (uint8_t i = 0; i < 4; i++) {
            pinMode(_banks[b].pins[i], OUTPUT);
            digitalWrite(_banks[b].pins[i], LOW);
        }
    }

    _hasSlow = false;
    for (uint8_t b = 0; b < _numBanks && !_hasSlow; b++)
        for (uint8_t c = 0; c < _banks[b].numChannels; c++)
            if (!isFastSlot(b, c)) { _hasSlow = true; break; }

    _bankIdx = 0;
    _chanIdx = 0;
    if (_hasSlow && isFastSlot(_bankIdx, _chanIdx)) advanceSlow();
    _scanComplete = false;
    _inFast = _numFast > 0;
    _fastIdx = 0;
    _fastPassOk = true;
}

void Scanner::update() {
    switch (_state) {
    case IDLE: {
        if (_inFast) {
            _curBank = _fast[_fastIdx].bank;
            _curChan = _fast[_fastIdx].ch;
        } else {
            _curBank = _bankIdx;
            _curChan = _chanIdx;
        }
        const MuxBank& bank = _banks[_curBank];
        _adc.setMux(bank.adcInput, MCP3561RT::Mux::AGND);
        selectMuxChannel(bank, _curChan);
        _timer = 0;
        _state = WAIT_MUX;
        break;
    }

    case WAIT_MUX:
        if (_timer >= T_MUX_SETTLE_US) {
            _adc.trigger();
            _timer = 0;
            _state = WAIT_CONV;
        }
        break;

    case WAIT_CONV:
        if (_adc.dataReady()) {
            int32_t raw;
            bool ok = _adc.readRaw(raw);
            if (ok) {
                _banks[_curBank].out[_curChan] =
                    _adc.vref() * (float(raw) / 8388608.0f);
            }
            finishSlot(ok);
        } else if (_timer >= ADC_CONV_TIMEOUT_US) {
            _timeouts++;
            finishSlot(false);
        }
        break;
    }
}

void Scanner::finishSlot(bool ok) {
    _state = IDLE;

    if (_inFast) {
        if (!ok) _fastPassOk = false;
        _fastIdx++;
        if (_fastIdx >= _numFast) {
            _fastIdx = 0;
            if (_fastPassOk) _fastSweeps++;
            _fastPassOk = true;
            // Hand one slot to the slow round-robin, if there is one.
            _inFast = !_hasSlow;
        }
        return;
    }

    advanceSlow();
    if (_numFast > 0) _inFast = true;
}

bool Scanner::isFastSlot(uint8_t bank, uint8_t ch) const {
    for (uint8_t i = 0; i < _numFast; i++)
        if (_fast[i].bank == bank && _fast[i].ch == ch) return true;
    return false;
}

void Scanner::selectMuxChannel(const MuxBank& bank, uint8_t ch) {
    for (uint8_t i = 0; i < 4; i++) {
        digitalWrite(bank.pins[i], (ch >> i) & 1);
    }
}

float Scanner::readBoardTemp() {
    // MCP3561RT internal temp sensor: set mux to TEMP_P vs TEMP_N,
    // do a blocking one-shot, convert using datasheet formula.
    // T(°C) = (V_temp - 0.492) / 0.00176 approximately.
    // This is a rough estimate — datasheet says ~±2°C typical.
    _adc.setMux(MCP3561RT::Mux::TEMP_P, MCP3561RT::Mux::TEMP_N);
    delayMicroseconds(T_MUX_SETTLE_US);
    _adc.trigger();

    elapsedMicros timeout;
    while (!_adc.dataReady()) {
        if (timeout > ADC_CONV_TIMEOUT_US) return -999.0f;
    }

    int32_t raw;
    if (!_adc.readRaw(raw)) return -999.0f;

    float voltage = _adc.vref() * (float(raw) / 8388608.0f);
    return (voltage - 0.492f) / 0.00176f;
}

void Scanner::advanceSlow() {
    if (!_hasSlow) return;
    // Step to the next channel that is not a fast slot. Bounded by the total
    // channel count, and _hasSlow guarantees at least one candidate exists.
    do {
        _chanIdx++;
        if (_chanIdx >= _banks[_bankIdx].numChannels) {
            _chanIdx = 0;
            _bankIdx++;
            if (_bankIdx >= _numBanks) {
                _bankIdx = 0;
                _scanComplete = true;
            }
        }
    } while (isFastSlot(_bankIdx, _chanIdx));
}
