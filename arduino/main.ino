#include "lib/MotorDriver/MotorDriver.h"
#include "lib/Communication/Command.h"

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

  uint8_t incoming = Serial.read();
  Serial.println(incoming);

  switch (incoming) {
    case Command::Forward:
      motorA.setPositive(MotorPinState::ACTIVE);
      motorA.setNegative(MotorPinState::INACTIVE);
      motorB.setPositive(MotorPinState::ACTIVE);
      motorB.setNegative(MotorPinState::INACTIVE);
      break;
    case Command::Right:
      motorA.setPositive(MotorPinState::ACTIVE);
      motorA.setNegative(MotorPinState::INACTIVE);
      motorB.setPositive(MotorPinState::INACTIVE);
      motorB.setNegative(MotorPinState::ACTIVE);
      break;
    case Command::Backward:
      motorA.setPositive(MotorPinState::INACTIVE);
      motorA.setNegative(MotorPinState::ACTIVE);
      motorB.setPositive(MotorPinState::INACTIVE);
      motorB.setNegative(MotorPinState::ACTIVE);
      break;
    case Command::Left:
      motorA.setPositive(MotorPinState::INACTIVE);
      motorA.setNegative(MotorPinState::ACTIVE);
      motorB.setPositive(MotorPinState::ACTIVE);
      motorB.setNegative(MotorPinState::INACTIVE);
      break;
    case Command::Stop:
      motorA.setPositive(MotorPinState::INACTIVE);
      motorA.setNegative(MotorPinState::INACTIVE);
      motorB.setPositive(MotorPinState::INACTIVE);
      motorB.setNegative(MotorPinState::INACTIVE);
      break;
  }
}