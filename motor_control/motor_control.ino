#include "src/MotorDriver/MotorDriver.h"
#include "src/Communication/Command.h"
#include "src/DistanceSensor/DistanceSensor.h"
#include "src/DiodeDriver/DiodeDriver.h"

#define MOTOR_A_ENABLE_PIN 10
#define MOTOR_A_POSITIVE_PIN 6 
#define MOTOR_A_NEGATIVE_PIN 5

#define MOTOR_B_ENABLE_PIN 9
#define MOTOR_B_POSITIVE_PIN 8
#define MOTOR_B_NEGATIVE_PIN 7

#define TRIG_PIN 4
#define ECHO_PIN 3

#define DIODE_PIN 2

#define SAFE_DISTANCE_IN_CM 10

#define MOTOR_POWER_STEP 85
#define MAX_MOTOR_POWER 255
#define MIN_MOTOR_POWER 85

#define SPEED_UP_ITER 3

MotorDriver motorA(MOTOR_A_POSITIVE_PIN, MOTOR_A_NEGATIVE_PIN, MOTOR_A_ENABLE_PIN);
MotorDriver motorB(MOTOR_B_POSITIVE_PIN, MOTOR_B_NEGATIVE_PIN, MOTOR_B_ENABLE_PIN);
DistanceSensor distanceSensor(TRIG_PIN, ECHO_PIN);
DiodeDriver diode(DIODE_PIN);
Command currentMovement = Command::Stop; 

uint8_t motorPower = MAX_MOTOR_POWER;

void increasePower();
void decreasePower();
void motorsSetPower(uint8_t power);
bool isNewMovement(Command command);
void speedUp();

void setup() {
  motorsSetPower(motorPower);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  auto incoming = static_cast<Command>(Serial.read());
  switch (incoming) {
    case Command::Forward:
      if (!isNewMovement(incoming)) {
        break;
      }
      motorsSetPower(0);
      delay(100);
      motorA.setPositive(MotorPinState::Active);
      motorA.setNegative(MotorPinState::Inactive);
      motorB.setPositive(MotorPinState::Active);
      motorB.setNegative(MotorPinState::Inactive);
      delay(100);
      speedUp();
      motorsSetPower(motorPower);
      break;
    case Command::Right:
      if (!isNewMovement(incoming)) {
        break;
      }
      motorsSetPower(0);
      delay(100);
      motorA.setPositive(MotorPinState::Active);
      motorA.setNegative(MotorPinState::Inactive);
      motorB.setPositive(MotorPinState::Inactive);
      motorB.setNegative(MotorPinState::Inactive);
      delay(100);
      speedUp();
      motorsSetPower(255);
      break;
    case Command::Backward:
      if (!isNewMovement(incoming)) {
        break;
      }
      motorsSetPower(0);
      delay(100);
      motorA.setPositive(MotorPinState::Inactive);
      motorA.setNegative(MotorPinState::Active);
      motorB.setPositive(MotorPinState::Inactive);
      motorB.setNegative(MotorPinState::Active);
      delay(100);
      speedUp();
      motorsSetPower(motorPower);
      break;
    case Command::Left:
      if (!isNewMovement(incoming)) {
        break;
      }
      motorsSetPower(0);
      delay(100);
      motorA.setPositive(MotorPinState::Inactive);
      motorA.setNegative(MotorPinState::Inactive);
      motorB.setPositive(MotorPinState::Active);
      motorB.setNegative(MotorPinState::Inactive);
      delay(100);
      speedUp();
      motorsSetPower(255);
      break;
    case Command::Stop:
      if (!isNewMovement(incoming)) {
        break;
      }
      motorsSetPower(0);
      delay(100);
      motorA.setPositive(MotorPinState::Inactive);
      motorA.setNegative(MotorPinState::Inactive);
      motorB.setPositive(MotorPinState::Inactive);
      motorB.setNegative(MotorPinState::Inactive);
      delay(100);
      motorsSetPower(motorPower);
      break;
    case Command::Slower:
      decreasePower();
      motorsSetPower(motorPower);
      Serial.println(motorPower);
      break;
    case Command::Faster:
      increasePower();
      motorsSetPower(motorPower);
      Serial.println(motorPower);
      break;
    case Command::GetSafeDistance: {
      long distance = distanceSensor.readDistance();
      if (distance > SAFE_DISTANCE_IN_CM) {
        Serial.print(true);
        break;
      }
      Serial.print(false);
      break;
    }
    case Command::LightOn:
      diode.activate();
      break;
    case Command::LightOff:
      diode.deactivate();
      break;
  }
}

void increasePower() {
  if ((motorPower + MOTOR_POWER_STEP) > MAX_MOTOR_POWER) {
    motorPower = MAX_MOTOR_POWER;
    return;
  }

  motorPower += MOTOR_POWER_STEP;
}

void decreasePower() {
  if (motorPower - MOTOR_POWER_STEP < MIN_MOTOR_POWER) {
    motorPower = MIN_MOTOR_POWER;
    return;
  }

  motorPower -= MOTOR_POWER_STEP;
}

void motorsSetPower(uint8_t power) {
  motorA.setPower(power);
  motorB.setPower(power);
}

bool isNewMovement(Command command) {
  if (currentMovement == command) {
    return false;
  }

  currentMovement = command;
  return true;
}

void speedUp() {
  for (int i = 0; i < SPEED_UP_ITER; i++) {
    motorsSetPower(255);
    delay(100);
    motorsSetPower(100);
    delay(100);
  }
}