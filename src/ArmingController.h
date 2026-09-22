#pragma once
#include <Arduino.h>

// Master arm line.
//
// PIN_ARM is a LEVEL, not a pulse: it must be held HIGH for as long as the
// board is armed, because it enables the solenoid drive stage. This matches
// test/dc_channel_test, which is the known-good reference for DC actuation:
// arm = PIN_ARM high, disarm = PIN_ARM low, nothing else.
//
// Power-on state is DISARMED (pin driven low in begin()).

class ArmingController {
public:
    explicit ArmingController(uint8_t armPin);

    // Drive the arm line low. Call early in setup().
    void begin();

    // Returns false if already in the requested state (the pin is re-driven
    // regardless, so a glitch can never leave it out of sync with isArmed()).
    bool arm();
    bool disarm();

    bool isArmed() const { return _armed; }

private:
    uint8_t _armPin;
    bool _armed = false;
};
