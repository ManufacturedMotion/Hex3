#include "toe.hpp"

Toe::Toe(bool useGPIO)
    : _gpio_enabled(useGPIO)
{}

bool Toe::begin()
{
    bool ok = sensor.begin();
    Serial.printf("sensor.begin returned %d\n", ok);
    if (!ok) {
        return false;
    }

    // Optional GPIO setup
    if (_gpio_enabled) {
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

bool Toe::isPressed()
{
    if (_gpio_enabled) {
        return digitalRead(TOE_PIN) == LOW; // Assuming active LOW for pressed
    }
    float distance = read();
    if (distance < 0.0f) {
        return false; // Sensor error, treat as not pressed
    }
    return distance <= toe_threshold;
}