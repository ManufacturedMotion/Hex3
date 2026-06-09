#include "toe.hpp"

Toe::Toe(bool useGPIO)
    : gpioEnabled(useGPIO)
{}

bool Toe::begin()
{
    if (!sensor.begin()) {
        return false;
    }

    // Optional GPIO setup
    if (gpioEnabled) {
        pinMode(TOE_PIN, INPUT_PULLUP);
    }

    return true;
}

float Toe::read()
{
    uint8_t range = sensor.readRange();

    if (sensor.readRangeStatus() != VL6180X_ERROR_NONE) {
        return -1.0f;
    }

    return static_cast<float>(range);
}