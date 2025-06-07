#pragma once
#include <Arduino.h>

class DiodeDriver {
    uint8_t _pin;

public:
    explicit DiodeDriver(uint8_t pin);

    void activate();
    void deactivate();
};
