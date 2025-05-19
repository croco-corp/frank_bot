#pragma once
#include <Arduino.h>

enum class MotorPinState: uint8_t {
    Active = HIGH,
    Inactive = LOW
};

class MotorDriver {
  uint8_t _positivePin;
  uint8_t _negativePin;
  uint8_t _enablePin;

public:
  explicit MotorDriver(uint8_t positivePin, uint8_t negativePin, uint8_t enablePin);

  void setPositive(MotorPinState state);
  void setNegative(MotorPinState state);
  void setPower(uint8_t power);
};