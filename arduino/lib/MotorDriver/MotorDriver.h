#pragma once
#include <Arduino.h>

namespace MotorPinState {
    constexpr bool ACTIVE = true;
    constexpr bool INACTIVE = false;
}

class MotorDriver {
  uint8_t _positivePin;
  uint8_t _negativePin;
  uint8_t _enablePin;

public:
  explicit MotorDriver(uint8_t positivePin, uint8_t negativePin, uint8_t enablePin);

  void setPositive(bool isActive);
  void setNegative(bool isActive);
  void setPower(uint8_t power);
};