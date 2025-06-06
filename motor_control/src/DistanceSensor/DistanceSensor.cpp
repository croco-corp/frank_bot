#include "DistanceSensor.h"

DistanceSensor::DistanceSensor() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

long DistanceSensor::readDistance() {
    long duration, distance;
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2); 
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH);

    distance = (duration/2) / 29.1;

    return distance;
}

bool DistanceSensor::isObstacle() {
    return readDistance() < 10;  // obstacle if closer than 10 cm
}