#include "src/MotorDriver/MotorDriver.h"
#include "src/Communication/Command.h"

#define MOTOR_A_ENABLE_PIN 9
#define MOTOR_A_POSITIVE_PIN 7
#define MOTOR_A_NEGATIVE_PIN 8

#define MOTOR_B_ENABLE_PIN 10
#define MOTOR_B_POSITIVE_PIN 5
#define MOTOR_B_NEGATIVE_PIN 6

MotorDriver motorA(MOTOR_A_POSITIVE_PIN, MOTOR_A_NEGATIVE_PIN, MOTOR_A_ENABLE_PIN);
MotorDriver motorB(MOTOR_B_POSITIVE_PIN, MOTOR_B_NEGATIVE_PIN, MOTOR_B_ENABLE_PIN);

void setup() {
  motorA.setPower(255);
  motorB.setPower(255);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  auto incoming = static_cast<Command>(Serial.read());

  switch (incoming) {
    case Command::Forward:
      motorA.setPositive(MotorPinState::Active);
      motorA.setNegative(MotorPinState::Inactive);
      motorB.setPositive(MotorPinState::Active);
      motorB.setNegative(MotorPinState::Inactive);
      break;
    case Command::Right:
      motorA.setPositive(MotorPinState::Active);
      motorA.setNegative(MotorPinState::Inactive);
      motorB.setPositive(MotorPinState::Inactive);
      motorB.setNegative(MotorPinState::Active);
      break;
    case Command::Backward:
      motorA.setPositive(MotorPinState::Inactive);
      motorA.setNegative(MotorPinState::Active);
      motorB.setPositive(MotorPinState::Inactive);
      motorB.setNegative(MotorPinState::Active);
      break;
    case Command::Left:
      motorA.setPositive(MotorPinState::Inactive);
      motorA.setNegative(MotorPinState::Active);
      motorB.setPositive(MotorPinState::Active);
      motorB.setNegative(MotorPinState::Inactive);
      break;
    case Command::Stop:
      motorA.setPositive(MotorPinState::Inactive);
      motorA.setNegative(MotorPinState::Inactive);
      motorB.setPositive(MotorPinState::Inactive);
      motorB.setNegative(MotorPinState::Inactive);
      break;
  }
}