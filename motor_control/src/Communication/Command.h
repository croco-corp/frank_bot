#include <Arduino.h>

enum class Command : uint8_t {
    Forward = 'w',
    Right = 'd',
    Backward = 's',
    Left = 'a',
    Stop = 'z',
    GetSafeDistance = 'x',
    Slower = 'c',
    Faster = 'v',
    LightOn = 'b',
    LightOff = 'n'
};