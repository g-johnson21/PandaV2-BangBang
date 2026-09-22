#include "SequenceHandler.h"
#include <cstring>
#include <cstdio>

SequenceHandler::SequenceHandler(MCP23S17& expander) : _exp(expander) {}

void SequenceHandler::begin() {
    // begin() configures every expander pin as an output and clears both OLATs.
    _exp.begin();
}

int SequenceHandler::parseChannel(char c) {
    int ch = -1;
    if (c >= '0' && c <= '9') ch = c - '0';
    else if (c >= 'A' && c <= 'F') ch = 10 + (c - 'A');
    else if (c >= 'a' && c <= 'f') ch = 10 + (c - 'a');
    return (ch >= 1 && ch <= NUM_ACTUATORS) ? ch : -1;
}

// One token: 's' <hex chan> <0|1> '.' <5 decimal digits>, nothing else.
bool SequenceHandler::parseToken(const char* tok, SequenceStep& out) {
    if (strlen(tok) != 9 || tok[0] != 's' || tok[3] != '.') return false;

    const int chan = parseChannel(tok[1]);
    if (chan < 0) return false;
    if (tok[2] != '0' && tok[2] != '1') return false;

    uint32_t delay = 0;
    for (uint8_t i = 4; i < 9; i++) {
        if (tok[i] < '0' || tok[i] > '9') return false;
        delay = delay * 10 + (tok[i] - '0');
    }

    out.channel = (uint8_t)chan;
    out.state = (tok[2] == '1');
    out.delay_ms = delay;
    return true;
}

bool SequenceHandler::setCommand(const char* command) {
    _numSteps = 0;
    _currentStep = 0;
    _active = false;
    _inDelay = false;
    _completed = false;
    _lastCmdValid = false;
    memset(_steps, 0, sizeof(_steps));

    // Work on a copy so strtok doesn't mutate the caller's buffer
    char buf[MAX_COMMAND_LEN + 1];
    size_t len = strlen(command);
    while (len > 0 && (command[len - 1] == '\n' || command[len - 1] == '\r'))
        len--;
    if (len == 0 || len > MAX_COMMAND_LEN) return false;
    memcpy(buf, command, len);
    buf[len] = '\0';

    SequenceStep parsed[NUM_MAX_COMMANDS];
    uint8_t count = 0;
    for (char* token = strtok(buf, ","); token; token = strtok(nullptr, ",")) {
        if (count >= NUM_MAX_COMMANDS) return false;
        if (!parseToken(token, parsed[count])) return false;
        count++;
    }
    if (count == 0) return false;

    memcpy(_steps, parsed, sizeof(SequenceStep) * count);
    _numSteps = count;
    memcpy(_lastCmd, command, len);
    _lastCmd[len] = '\0';
    _lastCmdValid = true;
    return true;
}

bool SequenceHandler::execute() {
    if (_numSteps == 0) return false;
    _currentStep = 0;
    _inDelay = false;
    _completed = false;
    _active = true;
    _stepTimer = 0;
    return true;
}

void SequenceHandler::update() {
    if (!_active || _numSteps == 0) return;
    if (_currentStep >= _numSteps) { _active = false; return; }

    if (!_inDelay) {
        SequenceStep& step = _steps[_currentStep];
        // setChannel() refuses a channel owned by bang-bang: writing it would
        // fight the controller and desync its cached valve state.
        const bool applied =
            setChannel(step.channel, step.state) == SetResult::OK;
        if (_onStep) _onStep(_currentStep, step, applied);
        _stepTimer = 0;
        _inDelay = true;
    }
    else if (_stepTimer >= _steps[_currentStep].delay_ms) {
        _inDelay = false;
        _currentStep++;

        if (_currentStep >= _numSteps) {
            _active = false;
            _currentStep = 0;
            _completed = true;
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

SequenceHandler::SetResult SequenceHandler::setChannel(uint8_t channel,
                                                      bool state) {
    if (channel < 1 || channel > NUM_ACTUATORS) return SetResult::RANGE;
    if (_isOwned && _isOwned(channel)) return SetResult::OWNED;
    _exp.setChannel(channel, state);
    return SetResult::OK;
}

bool SequenceHandler::setChannelRaw(uint8_t channel, bool state) {
    if (channel < 1 || channel > NUM_ACTUATORS) return false;
    return _exp.setChannel(channel, state);
}

uint8_t SequenceHandler::firstOwnedChannel() const {
    if (!_isOwned) return 0;
    for (uint8_t i = 0; i < _numSteps; i++)
        if (_isOwned(_steps[i].channel)) return _steps[i].channel;
    return 0;
}
