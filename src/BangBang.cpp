#include "BangBang.h"
#include <EEPROM.h>
#include <math.h>

// =============================================================================
// BBController implementation.
//
// State model:
//
//            ┌─────────────┐  enableSustain()         ┌──────────────┐
//            │  DISABLED   │ ───────────────────────▶ │   SUSTAIN    │
//            │ (press off, │                          │ (bang-bang)  │
//            │  vent off)  │ ◀────────── disableSustain/forceSafe ── │
//            └─────────────┘                          └──────┬───────┘
//               ▲    ▲                                       │
//               │    │  manualVentClose(pressure ok)         │ autovent_trigger hit
//               │    │                                       │ OR manualVent()
//               │    │                                       ▼
//       forceSafe(disarm)                            ┌──────────────┐
//               │                                    │  AUTO_VENT   │
//               │                                    │ (press off,  │
//               │                                    │  vent open)  │
//               │                                    └──────┬───────┘
//               │                                           │ latchAbort()
//               │                                           ▼
//               │                                    ┌──────────────┐
//               └──────────── forceSafe (disarm) ─── │    ABORT     │
//                                                    │  (latched)   │
//                                                    └──────────────┘
//
// Every transition emits an EVT: line. forceSafe() is the only path that
// clears an ABORT latch — there is no "unabort" GC command by design.
// =============================================================================

BBController::BBController(const float* ptArray,
                           uint8_t      pressPtIdx,
                           uint8_t      pressDcCh,
                           uint8_t      ventDcCh,
                           char         busId)
    : _ptArray(ptArray),
      _ptIdx(pressPtIdx),
      _pressCh(pressDcCh),
      _ventCh(ventDcCh),
      _busId(busId) {}

void BBController::bindIO(BBSetChannelFn setChannel, BBEmitFn emit) {
    _set  = setChannel;
    _emit = emit;
}

void BBController::_emitSafe(const char* cat, const char* detail) {
    if (!_emit) return;
    char buf[96];
    const float pt = _ptArray[_ptIdx];
    if (!detail || detail[0] == '\0')
        snprintf(buf, sizeof(buf), "pt=%.1f", pt);
    else if (strncmp(detail, "pt=", 3) == 0)
        snprintf(buf, sizeof(buf), "%s", detail);
    else
        snprintf(buf, sizeof(buf), "%s,pt=%.1f", detail, pt);
    _emit(cat, _busId, buf);
}

// ── Config ────────────────────────────────────────────────────────────────

void BBController::configureCore(float setpoint, float deadband, uint32_t waitMs, uint32_t maxOpenMs) {
    _cfg.setpoint_psi = setpoint;
    _cfg.deadband_psi = deadband;
    _cfg.wait_ms      = waitMs;
    _cfg.max_open_ms  = maxOpenMs;
    char buf[64];
    snprintf(buf, sizeof(buf), "sp=%.1f,db=%.1f,wait=%lu,maxOpen=%lu",
             setpoint, deadband, (unsigned long)waitMs, (unsigned long)maxOpenMs);
    _emitSafe("CFG_PUSH", buf);
}

void BBController::configurePredictiveClose(uint32_t closeDelayMs) {
    _cfg.close_delay_ms = closeDelayMs;
    char buf[40];
    snprintf(buf, sizeof(buf), "closeDelay=%lu",
             (unsigned long)closeDelayMs);
    _emitSafe("CFG_PUSH", buf);
}

void BBController::configureVent(float triggerPsi, bool autoVentEnabled) {
    _cfg.autovent_trigger = triggerPsi;
    _cfg.autovent_enabled = autoVentEnabled;
    char buf[48];
    snprintf(buf, sizeof(buf), "avTrig=%.1f,avAuto=%d", triggerPsi, autoVentEnabled ? 1 : 0);
    _emitSafe("CFG_PUSH", buf);
}

void BBController::setPredictiveEnabled(bool enabled) {
    if (_predictiveEnabled == enabled) return;
    _predictiveEnabled = enabled;
    _emitSafe("PRED_MODE", enabled ? "enabled=1" : "enabled=0");
}

// ── State transitions ─────────────────────────────────────────────────────

void BBController::_setPress(bool open, const char* reason) {
    if (open == _pressOpen) return;
    if (_set) _set(_pressCh, open);
    _pressOpen = open;
    if (open) _openTimer = 0;
    char buf[48];
    snprintf(buf, sizeof(buf), "press=%d,reason=%s", open ? 1 : 0, reason ? reason : "");
    _emitSafe("VALVE", buf);
}

void BBController::_setVent(bool open, const char* reason) {
    if (!hasVentHw()) return;
    if (open == _ventOpen) return;
    if (_set) _set(_ventCh, open);
    _ventOpen = open;
    char buf[48];
    snprintf(buf, sizeof(buf), "vent=%d,reason=%s", open ? 1 : 0, reason ? reason : "");
    _emitSafe("VALVE", buf);
}

void BBController::_goto(BBState next, const char* reason) {
    if (next == _state) return;
    const char* evt = "STATE";
    switch (next) {
        case BBState::DISABLED:  evt = "BB_OFF";       break;
        case BBState::SUSTAIN:   evt = "BB_ON";        break;
        case BBState::AUTO_VENT: evt = "AV_ENTER";     break;
        case BBState::ABORT:     evt = "ABORT_ENTER";  break;
    }
    _state = next;
    _switchTimer = 0;
    _emitSafe(evt, reason);
}

bool BBController::enableSustain() {
    if (_state != BBState::DISABLED) {
        _emitSafe("OWN_CONFLICT", "enable while non-disabled");
        return false;
    }
    _goto(BBState::SUSTAIN, "GC enable");
    return true;
}

void BBController::disableSustain() {
    if (_state == BBState::ABORT) {
        _setVent(false, "abort clear");
        _abortLatched = false;
        _goto(BBState::DISABLED, "abort clear (b0)");
        _emitSafe("ABORT_CLEAR", "operator b0");
        return;
    }
    _setPress(false, "disable");
    // Do not touch vent — if we were in AUTO_VENT, caller must manualVentClose.
    if (_state == BBState::SUSTAIN) {
        _goto(BBState::DISABLED, "GC disable");
    }
}

bool BBController::manualVent() {
    if (!hasVentHw()) {
        _emitSafe("AV_NO_HW", "vent DC channel unset");
        return false;
    }
    if (_state == BBState::ABORT) {
        _emitSafe("OWN_CONFLICT", "manualVent ignored while ABORT latched");
        return false;
    }
    _setPress(false, "manualVent");
    _setVent(true, "manualVent");
    _goto(BBState::AUTO_VENT, "manualVent");
    return true;
}

bool BBController::manualVentClose(bool force) {
    if (_state != BBState::AUTO_VENT) return false;
    if (!force) {
        const float hi = _cfg.setpoint_psi + _cfg.deadband_psi * 0.5f;
        if (_lastPressure > hi) {
            _emitSafe("AV_REJECT_CLOSE", "pressure still above deadband-high");
            return false;
        }
    }
    _setVent(false, "manualVentClose");
    _goto(BBState::DISABLED, force ? "manualVentClose(force)" : "manualVentClose");
    _emitSafe("AV_EXIT", force ? "forced" : "pressure ok");
    return true;
}

bool BBController::latchAbort() {
    if (!hasVentHw()) {
        _emitSafe("AV_NO_HW", "abort requested but vent unset");
        // Still close press so we have at least *some* safing.
        _setPress(false, "abort-no-hw");
        _abortLatched = true;
        _goto(BBState::ABORT, "abort (no vent HW)");
        return false;
    }
    _setPress(false, "abort");
    _setVent(true, "abort");
    _abortLatched = true;
    _goto(BBState::ABORT, "latchAbort");
    return true;
}

void BBController::forceSafe() {
    _setPress(false, "forceSafe");
    _setVent(false, "forceSafe");
    setPredictiveEnabled(false);
    bool wasLatched = _abortLatched;
    _abortLatched = false;
    if (_state != BBState::DISABLED) {
        _goto(BBState::DISABLED, wasLatched ? "disarm (ABORT cleared)" : "forceSafe");
    }
    if (wasLatched) _emitSafe("ABORT_CLEAR", "disarm");
}

// ── Main loop tick ────────────────────────────────────────────────────────

void BBController::update(bool armed, bool psiSettled, uint32_t pressureSampleMs) {
    // Always snapshot live PT for the 1 Hz BB heartbeat GC reads, even when
    // disarmed or DISABLED — lastPressure must not stick at 0 across disarm.
    _lastPressure = _ptArray[_ptIdx];
    _updatePressureRate(pressureSampleMs);

    if (!armed) {
        if (_state != BBState::DISABLED || _pressOpen || _ventOpen || _abortLatched) {
            forceSafe();
        }
        return;
    }

    // Sanity bounds apply in every non-DISABLED, non-ABORT state. ABORT is
    // already latched safe (valves parked) and only forceSafe()/disarm clears
    // it — re-checking here would just re-emit SANITY_FAIL/latchAbort every
    // tick forever, flooding the telemetry stream and starving Serial2 TX.
    // Also defer until the median filter has filled — early samples are not
    // representative and must not trip SANITY_FAIL on enable.
    if (psiSettled &&
        _state != BBState::DISABLED && _state != BBState::ABORT) {
        if (isnanf(_lastPressure) ||
            _lastPressure < BB_PRESSURE_MIN_PSI ||
            _lastPressure > BB_PRESSURE_MAX_PSI) {
            _emitSafe("SANITY_FAIL", "out of bounds");
            latchAbort();
            return;
        }
    }

    switch (_state) {
        case BBState::DISABLED:                       break;
        case BBState::SUSTAIN:    _updateSustain();   break;
        case BBState::AUTO_VENT:  _updateAutoVent();  break;
        case BBState::ABORT:      _updateAbort();     break;
    }
}

void BBController::_updatePressureRate(uint32_t sampleMs) {
    // update() runs much faster than new PT sweeps complete. Use the frame
    // timestamp so repeated reads of the same pressure do not pull the
    // derivative toward zero.
    if (sampleMs == 0) {
        _rateValid = false;
        _pressureRate = 0.0f;
        _projectedPressure = _lastPressure;
        _predictionHorizonMs = 0.0f;
        _sampleIntervalMs = 0.0f;
        _rateSampleMs = 0;
        return;
    }
    if (sampleMs == _rateSampleMs) return;

    if (!isfinite(_lastPressure)) {
        _rateValid = false;
        _pressureRate = 0.0f;
        _projectedPressure = _lastPressure;
        _rateSampleMs = sampleMs;
        _rateSamplePressure = _lastPressure;
        return;
    }

    if (_rateSampleMs == 0) {
        _rateSampleMs = sampleMs;
        _rateSamplePressure = _lastPressure;
        _projectedPressure = _lastPressure;
        return;
    }

    const uint32_t dtMs = sampleMs - _rateSampleMs;
    if (dtMs == 0 || dtMs > BB_RATE_RESET_MS) {
        _rateValid = false;
        _pressureRate = 0.0f;
        _sampleIntervalMs = 0.0f;
    } else {
        if (_sampleIntervalMs > 0.0f) {
            _sampleIntervalMs +=
                BB_PRESSURE_RATE_ALPHA * ((float)dtMs - _sampleIntervalMs);
        } else {
            _sampleIntervalMs = (float)dtMs;
        }
        const float instantaneousRate =
            (_lastPressure - _rateSamplePressure) * (1000.0f / (float)dtMs);
        if (isfinite(instantaneousRate)) {
            if (_rateValid) {
                _pressureRate +=
                    BB_PRESSURE_RATE_ALPHA * (instantaneousRate - _pressureRate);
            } else {
                _pressureRate = instantaneousRate;
                _rateValid = true;
            }
        } else {
            _rateValid = false;
            _pressureRate = 0.0f;
        }
    }

    _rateSampleMs = sampleMs;
    _rateSamplePressure = _lastPressure;
    const float risingRate =
        (_rateValid && _pressureRate > 0.0f) ? _pressureRate : 0.0f;
    const float medianDelayMs =
        0.5f * (float)(PT_PSI_MEDIAN_WINDOW - 1) * _sampleIntervalMs;
    _predictionHorizonMs = (float)_cfg.close_delay_ms + medianDelayMs;
    _projectedPressure =
        _lastPressure + risingRate * (_predictionHorizonMs / 1000.0f);
}

void BBController::_updateSustain() {
    // Auto-vent trigger takes precedence over bang-bang.
    if (_cfg.autovent_enabled &&
        hasVentHw() &&
        _lastPressure > _cfg.autovent_trigger) {
        _setPress(false, "autovent trigger");
        _setVent(true,  "autovent trigger");
        _goto(BBState::AUTO_VENT, "autoTrigger");
        return;
    }

    const float hi = _cfg.setpoint_psi + _cfg.deadband_psi * 0.5f;
    const float lo = _cfg.setpoint_psi - _cfg.deadband_psi * 0.5f;

    // Slow-press: if max_open_ms > 0 and press has been open for that long,
    // force a close and enforce wait_ms before another open is allowed.
    if (_pressOpen && _cfg.max_open_ms > 0 && _openTimer >= _cfg.max_open_ms) {
        _setPress(false, "slowPress cap");
        _switchTimer = 0;
        return;
    }

    // Closing must never be held off by wait_ms: with a high-pressure source,
    // the prediction can cross deadband-high shortly after opening. wait_ms
    // is enforced below only as the closed dwell before another opening.
    if (_predictiveEnabled && _pressOpen && _rateValid && _pressureRate > 0.0f &&
        _projectedPressure >= hi) {
        char buf[88];
        snprintf(buf, sizeof(buf),
                 "rate=%.1f,pred=%.1f,hi=%.1f,delay=%lu,horizon=%.1f",
                 _pressureRate, _projectedPressure, hi,
                 (unsigned long)_cfg.close_delay_ms, _predictionHorizonMs);
        _emitSafe("PRED_CLOSE", buf);
        _setPress(false, "predictive cutoff");
        _switchTimer = 0;
    } else if (_lastPressure >= hi && _pressOpen) {
        _setPress(false, "at/above hi");
        _switchTimer = 0;
    } else if (!_pressOpen && _switchTimer >= _cfg.wait_ms &&
               _lastPressure < lo) {
        _setPress(true, "below lo");
        _switchTimer = 0;
    }
}

void BBController::_updateAutoVent() {
    // Exit condition: pressure has come back to or below deadband-high.
    const float hi = _cfg.setpoint_psi + _cfg.deadband_psi * 0.5f;
    if (_lastPressure <= hi) {
        _setVent(false, "autovent exit");
        _emitSafe("AV_EXIT", "pressure below hi");
        // Return to SUSTAIN if autovent was auto-triggered and operator wants
        // continued control. Conservative default: drop to DISABLED so the
        // operator must explicitly re-arm BB. Safer, requires an explicit
        // decision before we start driving valves again.
        _goto(BBState::DISABLED, "AV_EXIT→DISABLED");
    }
}

void BBController::_updateAbort() {
    // Latched — nothing to do. Valves were set at latchAbort(). Only
    // forceSafe() (triggered by disarm) clears this state.
}

// ── EEPROM persistence ────────────────────────────────────────────────────

static uint8_t computeCrc(const BBConfig& lox, const BBConfig& fuel) {
    uint8_t crc = 0;
    const uint8_t* p;
    p = reinterpret_cast<const uint8_t*>(&lox);
    for (size_t i = 0; i < sizeof(BBConfig); i++) crc ^= p[i];
    p = reinterpret_cast<const uint8_t*>(&fuel);
    for (size_t i = 0; i < sizeof(BBConfig); i++) crc ^= p[i];
    return crc;
}

void bbLoadEeprom(BBController& lox, BBController& fuel) {
    BBEepromBlock block;
    EEPROM.get(BB_EEPROM_ADDR, block);

    if (block.magic != BB_EEPROM_MAGIC) return;
    if (block.crc != computeCrc(block.lox, block.fuel)) return;

    lox.configureCore(block.lox.setpoint_psi, block.lox.deadband_psi,
                      block.lox.wait_ms, block.lox.max_open_ms);
    lox.configurePredictiveClose(block.lox.close_delay_ms);
    lox.configureVent(block.lox.autovent_trigger, block.lox.autovent_enabled);

    fuel.configureCore(block.fuel.setpoint_psi, block.fuel.deadband_psi,
                       block.fuel.wait_ms, block.fuel.max_open_ms);
    fuel.configurePredictiveClose(block.fuel.close_delay_ms);
    fuel.configureVent(block.fuel.autovent_trigger, block.fuel.autovent_enabled);
}

void bbSaveEeprom(const BBController& lox, const BBController& fuel) {
    BBEepromBlock block;
    block.magic = BB_EEPROM_MAGIC;
    block.lox   = lox.config();
    block.fuel  = fuel.config();
    block.crc   = computeCrc(block.lox, block.fuel);
    EEPROM.put(BB_EEPROM_ADDR, block);
}
