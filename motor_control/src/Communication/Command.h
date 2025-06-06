#include <Arduino.h>

enum class Command : uint8_t {
    Forward = 'w',
    Right = 'd',
    Backward = 's',
    Left = 'a',
    Stop = 'z',
    GetSafeDistance = 'x'
};