#include "DistanceSensor.h"
#include <Arduino.h>

DistanceSensor::DistanceSensor(uint8_t trigPin, uint8_t echoPin)
    : _trigPin(trigPin), _echoPin(echoPin) {
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
}

long DistanceSensor::readDistance() {
    long duration, distance;
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2); 
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    duration = pulseIn(_echoPin, HIGH);

    distance = (duration/2) / 29.1;

    return distance;
}
