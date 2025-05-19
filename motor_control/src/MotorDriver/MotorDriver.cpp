#include "MotorDriver.h"
#include <Arduino.h>

MotorDriver::MotorDriver(uint8_t positivePin, uint8_t negativePin, uint8_t enablePin)
  : _positivePin(positivePin), _negativePin(negativePin), _enablePin(enablePin) {
  pinMode(_positivePin, OUTPUT);
  pinMode(_negativePin, OUTPUT);
  pinMode(_enablePin, OUTPUT);

  digitalWrite(_positivePin, LOW);
  digitalWrite(_negativePin, LOW);
  digitalWrite(_enablePin, LOW);
}

void MotorDriver::setPositive(MotorPinState state) {
  auto state_number = static_cast<uint8_t>(state);
  digitalWrite(_positivePin, state_number);
}

void MotorDriver::setNegative(MotorPinState state) {
  auto state_number = static_cast<uint8_t>(state);
  digitalWrite(_negativePin, state_number);
}

void MotorDriver::setPower(uint8_t power) {
  analogWrite(_enablePin, power);
}