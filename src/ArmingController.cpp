#include "ArmingController.h"

ArmingController::ArmingController(uint8_t armPin) : _armPin(armPin) {}

void ArmingController::begin() {
    pinMode(_armPin, OUTPUT);
    digitalWrite(_armPin, LOW);
    _armed = false;
}

bool ArmingController::arm() {
    const bool changed = !_armed;
    digitalWrite(_armPin, HIGH);
    _armed = true;
    return changed;
}

bool ArmingController::disarm() {
    const bool changed = _armed;
    digitalWrite(_armPin, LOW);
    _armed = false;
    return changed;
}
