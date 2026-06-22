#include "toe.hpp"
#include <Wire.h>

Toe::Toe(){}

bool Toe::begin()
{
    Wire.setTimeout(2, true);
    bool ok = sensor.begin();
    Serial.printf("toe idle is %f\n", toe_idle);
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
    _initialized = true;
    return true;
}

void Toe::update()
{

    if (!_initialized)
    {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("[Toe] WARNING: update() called but not initialized, returning toe_idle");
        #endif
        _last_range = toe_idle;
        return;
    }

    if (millis() - _last_probe_ms >= _probe_interval_ms)
    {
        _last_probe_ms = millis();
        _connected = probeSensor();
        if (!_connected)
        {
            #if LOG_LEVEL >= BASIC_DEBUG
                Serial.println("[Toe] WARNING: sensor probe failed, sensor may be disconnected");
            #endif
            _last_range = toe_idle; // reset to idle on probe failure
            _first_read = false; //reset first read so we get smooth readings if sensor recovers
            return;
        }
    }

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
    if(!_connected)
    {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("[Toe] WARNING: update() called but sensor not connected, returning toe_idle");
        #endif
        _last_range = toe_idle; // return to idle if not connected
        _first_read = false; //reset first read so we get smooth readings if sensor recovers
        return;
    }

    if (!_first_read)
    {
        // first valid reading, initialize _last_range directly to avoid startup spikes
        _last_range = new_val;
        _first_read = true;
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

    if (CALIBRATING_TOE)
    {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.printf("[Toe] WARNING: read() called but still calibrating, returning current average, %f", _last_range);
        #endif
        return _last_range; // return current average during calibration
    }

    if (!_first_read){
        #if LOG_LEVEL >= BASIC_DEBUG
                Serial.println("[Toe] WARNING: read() called but no valid reading yet, returning toe_idle");
        #endif
        _last_range = toe_idle;
        _first_read = true;
        return _last_range;
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

//periodically check this to confirm if sensor is still operational
bool Toe::probeSensor()
{
    Wire.beginTransmission(0x29); // VL6180X default address
    return Wire.endTransmission() == 0;
}