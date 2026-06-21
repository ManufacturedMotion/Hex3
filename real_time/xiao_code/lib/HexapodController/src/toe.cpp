#include "toe.hpp"
#include <Wire.h>

Toe::Toe(){}

bool Toe::begin()
{
    Wire.setTimeout(2, true);
    bool ok = sensor.begin();

    #if LOG_LEVEL >= BASIC_DEBUG
        Serial.printf("[Toe] sensor.begin returned: %d\n", ok);
    #endif

    if (!ok)
    {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("[Toe] ERROR: sensor.begin failed");
        #endif
        return false;
    }
    sensor.startRangeContinuous(10);
    return true;
}

void Toe::update()
{
    uint8_t raw = sensor.readRangeResult();
    if (sensor.readRangeStatus() != VL6180X_ERROR_NONE)
        return;

    //TODO - add logic for getting an unfilterd value for calibration 
    float new_val = (float)raw;
    // EMA filter
    if (!_valid)
    {
        //first value / reading does not get filtering - important to set initial value
        _last_range = new_val;
        _valid = true;
    }
    else
    {
        _last_range = 0.6f * _last_range + 0.4f * new_val;
    }
}

float Toe::read()
{
    if (!_valid)
    {
        #if LOG_LEVEL >= BASIC_DEBUG
             Serial.println("[Toe] WARNING: read() called but no valid reading yet, returning toe_idle");
        #endif
        return toe_idle;
    }
    #if LOG_LEVEL >= BASIC_DEBUG
        Serial.printf("[Toe] read() -> %f\n", _last_range);
    #endif
    return _last_range;
}

bool Toe::isPressed()
{
    float distance = read();
    if (distance < 0.0f) //TODO handle error state better
        return false;

    //TODO toe_threshold should be a percentaeg of toe_idle
    return distance <= toe_threshold;
}