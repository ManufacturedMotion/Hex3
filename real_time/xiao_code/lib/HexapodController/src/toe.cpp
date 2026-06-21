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

    float new_val = (float)raw;
    //Serial.printf("[Toe] raw range: %d\n", raw);
    // -------------------------
    // CALIBRATION MODE -- collects samples to determine toe_idle at startup. User will copy stable reading to config.hpp TOE_IDLE_READ for future runs.
    // -------------------------
    if (CALIBRATING_TOE)
    {
        // ring buffer sum approach
        if (_calib_count == CALIB_SAMPLES)
        {
            _calib_sum -= _calib_buffer[_calib_index];
        }
        _calib_buffer[_calib_index] = new_val;
        _calib_sum += new_val;
        _calib_index = (_calib_index + 1) % CALIB_SAMPLES;
        if (_calib_count < CALIB_SAMPLES)
            _calib_count++;
        if (_calib_count == CALIB_SAMPLES)
        {
            float avg = _calib_sum / CALIB_SAMPLES;
            _last_range = avg;
            #if LOG_LEVEL >= BASIC_DEBUG
                Serial.printf("[Toe CAL] avg=%.2f (last_range updated)\n", avg);
            #endif
        }

        return;
    }

    // -------------------------
    // NORMAL MODE -- EMA filter approach
    // -------------------------
    if (!_valid)
    {
        // first valid reading, initialize _last_range directly to avoid startup spikes
        _last_range = new_val;
        _valid = true;
    }
    else
    {
        //_last_range = 0.2f * _last_range + 0.8f * new_val;
        if (new_val < _last_range)
            _last_range = 0.5f * _last_range + 0.5f * new_val; // fast drop
        else
            _last_range = 0.9f * _last_range + 0.1f * new_val; // slow rise
    }
}

float Toe::read()
{
    if (!_valid)
    {
        if (CALIBRATING_TOE)
        {
            #if LOG_LEVEL >= BASIC_DEBUG
                Serial.printf("[Toe] WARNING: read() called but still calibrating, returning current average, %f", _last_range);
            #endif
            return _last_range; // return current average during calibration
        }

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
    bool pressed = distance <= toe_idle * toe_threshold;
    #if LOG_LEVEL >= BASIC_DEBUG
        if (pressed)
            Serial.printf("[Toe] isPressed() -> distance: %f, threshold: %f, pressed: %d\n", distance, toe_threshold, pressed);
    #endif  
    return pressed;
}