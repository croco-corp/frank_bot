#include "DiodeDriver.h"
#include <Arduino.h>

DiodeDriver::DiodeDriver(uint8_t pin)
: _pin(pin) {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
}

void DiodeDriver::activate() {
    digitalWrite(_pin, HIGH);
}

void DiodeDriver::deactivate() {
    digitalWrite(_pin, LOW);
}
