#pragma once 
#include <Wire.h> 

class DistanceSensor{
    uint8_t _trigPin;
    uint8_t _echoPin;
public:
    explicit DistanceSensor(uint8_t trigPin, uint8_t echoPin);
    long readDistance();
};