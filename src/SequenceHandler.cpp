#include "SequenceHandler.h"
#include <cstring>
#include <cstdio>

SequenceHandler::SequenceHandler(MCP23S17& expander) : _exp(expander) {}

void SequenceHandler::begin() {
    // begin() configures every expander pin as an output and clears both OLATs.
    _exp.begin();
}

bool SequenceHandler::setCommand(const char* command) {
    _numSteps = 0;
    _currentStep = 0;
    _active = false;
    _inDelay = false;
    _lastCmdValid = false;
    memset(_steps, 0, sizeof(_steps));

    // Work on a copy so strtok doesn't mutate the caller's buffer
    char buf[256];
    strncpy(buf, command, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    // Strip trailing newline/carriage return
    size_t len = strlen(buf);
    while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r'))
        buf[--len] = '\0';

    char* token = strtok(buf, ",");
    while (token && _numSteps < NUM_MAX_COMMANDS) {
        // Each token starts with 's', skip it
        if (token[0] != 's') {
            token = strtok(nullptr, ",");
            continue;
        }

        unsigned chan, state, delay;
        if (sscanf(token + 1, "%1x%1u.%5u", &chan, &state, &delay) != 3) {
            _numSteps = 0;
            return false;
        }

        // Validate ranges
        if (chan < 1 || chan > NUM_ACTUATORS || state > 1) {
            _numSteps = 0;
            return false;
        }

        _steps[_numSteps].channel = chan;
        _steps[_numSteps].state = state;
        _steps[_numSteps].delay_ms = delay;
        _numSteps++;

        token = strtok(nullptr, ",");
    }

    if (_numSteps > 0) {
        strncpy(_lastCmd, command, sizeof(_lastCmd) - 1);
        _lastCmd[sizeof(_lastCmd) - 1] = '\0';
        // Strip trailing whitespace from saved command
        len = strlen(_lastCmd);
        while (len > 0 && (_lastCmd[len-1] == '\n' || _lastCmd[len-1] == '\r'))
            _lastCmd[--len] = '\0';
        _lastCmdValid = true;
        return true;
    }
    return false;
}

bool SequenceHandler::execute() {
    if (_numSteps == 0) return false;
    _currentStep = 0;
    _inDelay = false;
    _active = true;
    _stepTimer = 0;
    return true;
}

void SequenceHandler::update() {
    if (!_active || _numSteps == 0) return;
    if (_currentStep >= _numSteps) { _active = false; return; }

    if (!_inDelay) {
        SequenceStep& step = _steps[_currentStep];
        _exp.setChannel(step.channel, step.state);
        if (_onStep) _onStep(_currentStep, step);
        _stepTimer = 0;
        _inDelay = true;
    }
    else if (_stepTimer >= _steps[_currentStep].delay_ms) {
        _inDelay = false;
        _currentStep++;

        if (_currentStep >= _numSteps) {
            _active = false;
            _currentStep = 0;
        }
    }
}

void SequenceHandler::cancelExecution() {
    _active = false;
    _inDelay = false;
    _currentStep = 0;
}

void SequenceHandler::setAllOff() {
    // Clears all 16 expander outputs, not just the exposed NUM_ACTUATORS —
    // this is the disarm path, so nothing should be left driven.
    _exp.allOff();
}

bool SequenceHandler::setChannel(uint8_t channel, bool state) {
    if (channel < 1 || channel > NUM_ACTUATORS) return false;
    return _exp.setChannel(channel, state);
}
