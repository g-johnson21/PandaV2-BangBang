#pragma once
#include <Arduino.h>
#include <MCP23S17.h>
#include "BoardConfig.h"

// Timed actuator sequence engine.
// Parses V1-compatible sequence strings, drives solenoids through the MCP23S17
// I/O expander (channel n → ACTUATEn).
// Format: "s<chan_hex><state>.<delay_ms_5dig>,..."
// Example: "s11.01000,s10.00000,s81.02000,s80.00000"
//
// Parsing is all-or-nothing: every token must match the format exactly (one
// hex digit, 0/1, '.', exactly five decimal digits). An over-long command or
// too many steps is rejected rather than truncated, so a cut-off token can
// never be run with a shortened delay.

struct SequenceStep {
    uint8_t channel; // 1-indexed (1..NUM_ACTUATORS)
    bool state;
    uint32_t delay_ms;
};

class SequenceHandler {
public:
    // Called for each step as it is reached, before its delay runs. `applied`
    // is false when the step was skipped because its channel is owned.
    using StepFn = void (*)(uint8_t index, const SequenceStep& step, bool applied);

    // Returns true if a channel is currently reserved by another controller
    // (bang-bang). Steps on such channels are skipped, never written.
    using OwnedFn = bool (*)(uint8_t channel);

    // Longest accepted command, including the leading 's'.
    static constexpr size_t MAX_COMMAND_LEN = RS485_RX_BUF - 1;

    // The expander must outlive this handler — main.cpp owns it as a global.
    explicit SequenceHandler(MCP23S17& expander);

    // Bring the expander up with all outputs off. Call from setup() after
    // SPI1.begin().
    void begin();

    // Parse a sequence command string. Returns false on parse error.
    bool setCommand(const char* command);

    // Execute the loaded sequence. Returns false if nothing loaded.
    bool execute();

    // Call from main loop — advances the state machine.
    void update();

    void cancelExecution();
    void setAllOff();

    // Direct single-channel control (bypasses sequencing)
    bool setChannel(uint8_t channel, bool state);

    void setStepCallback(StepFn fn) { _onStep = fn; }
    void setOwnershipCheck(OwnedFn fn) { _isOwned = fn; }

    // True once after a sequence runs to its last step. Cancellation and
    // replacement never set it, so an aborted run is not reported as complete.
    bool takeCompleted() {
        const bool c = _completed;
        _completed = false;
        return c;
    }

    bool isActive() const { return _active; }
    bool hasSequence() const { return _numSteps > 0; }
    uint8_t getNumSteps() const { return _numSteps; }
    const SequenceStep& getStep(uint8_t i) const { return _steps[i]; }
    const char* getLastCommand() const { return _lastCmdValid ? _lastCmd : ""; }

private:
    MCP23S17& _exp;
    StepFn _onStep = nullptr;
    OwnedFn _isOwned = nullptr;
    SequenceStep _steps[NUM_MAX_COMMANDS];
    uint8_t _numSteps = 0;
    uint8_t _currentStep = 0;
    bool _active = false;
    bool _inDelay = false;
    bool _completed = false;
    elapsedMillis _stepTimer;

    bool _lastCmdValid = false;
    char _lastCmd[MAX_COMMAND_LEN + 1] = {0};

    static bool parseToken(const char* tok, SequenceStep& out);
};
