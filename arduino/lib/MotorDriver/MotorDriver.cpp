#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t positivePin, uint8_t negativePin, uint8_t enablePin)
  : _positivePin(positivePin), _negativePin(negativePin), _enablePin(enablePin) {
  pinMode(_positivePin, OUTPUT);
  pinMode(_negativePin, OUTPUT);
  pinMode(_enablePin, OUTPUT);

  digitalWrite(_positivePin, LOW);
  digitalWrite(_negativePin, LOW);
  digitalWrite(_enablePin, LOW);
}

void MotorDriver::setPositive(bool is_active) {
  int8_t pinState = LOW;
  if (is_active) {
    pinState = HIGH;
  }

  digitalWrite(_positivePin, pinState);
}

void MotorDriver::setNegative(bool is_active) {
  int8_t pinState = LOW;
  if (is_active) {
    pinState = HIGH;
  }

  digitalWrite(_negativePin, pinState);
}

void MotorDriver::setPower(uint8_t power) {
  analogWrite(_enablePin, power);
}