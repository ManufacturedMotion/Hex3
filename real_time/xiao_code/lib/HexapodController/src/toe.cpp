#include "toe.hpp"
#include <Wire.h>
#include "log_levels.hpp"

Toe::Toe(){}

bool Toe::begin()
{
    Wire.setTimeout(20, true);
    bool ok = sensor.begin();
    Serial.printf("toe idle is %f\n", toe_idle);
    #if LOG_LEVEL >= BASIC_DEBUG
        Serial.printf("[Toe] sensor.begin returned: %d\n", ok);
    #endif

    if (!ok)
    {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("[Toe] ERROR: sensor.begin failed, check wiring and I2C address");
        #endif
        return false;
    }

    delay(50);  // IMPORTANT: let sensor boot fully
    // hard reset continuous state
    sensor.stopRangeContinuous();  
    delay(50);
    sensor.startRangeContinuous(10);
    delay(50);  // allow the first continuous reading to become available
    state = ToeState::OPERATING;
    return true;
}

void Toe::update()
{
    // #if LOG_LEVEL >= BASIC_DEBUG
        Serial.printf("[Toe] update() called, state=%d\n", static_cast<int>(state));
    // #endif
    switch(state)
    {
        case ToeState::UNINITIALIZED:
            #if LOG_LEVEL >= BASIC_DEBUG
                Serial.println("[Toe] WARNING: update() called but not initialized, returning toe_idle");
            #endif
            _last_range = toe_idle;
            break;

        case ToeState::OPERATING:
        {
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
            
            if (millis() - _last_read_ms > _read_interval_ms)
            {
                if (!sensor.isRangeComplete()) {
                    // #if LOG_LEVEL >= BASIC_DEBUG
                        Serial.println("[Toe] WARNING: sensor range not complete, skipping read");
                    // #endif
                    _unchanged_count++;
                    if (_unchanged_count >= TOE_MAX_UNCHANGED_COUNT) {
                        bool ok = sensor.begin();
                        if (!ok) {
                            #if LOG_LEVEL >= BASIC_DEBUG
                                Serial.println("[Toe] ERROR: sensor.begin failed during reset, check wiring and I2C address");
                            #endif
                            state = ToeState::ERROR;
                            return;
                        }
                        
                        state = ToeState::RESET_1;
                        _last_action_time = millis();
                        _unchanged_count = 0;
                    }
                    return;
                }
                _last_read_ms = millis();

                
                uint8_t raw = medianFilter(sensor.readRangeResult());
                float new_val = (float)raw;
                if (!_first_read)
                {
                    // first valid reading, initialize _last_range directly to avoid startup spikes
                    _last_range = new_val;
                    _first_read = true;
                }
                else
                {
                    if (fabs(new_val - _last_range) < 0.001f) {
                        _unchanged_count++;
                    } else {
                        _unchanged_count = 0;
                    }
                    if (_unchanged_count >= TOE_MAX_UNCHANGED_COUNT) {
                        bool ok = sensor.begin();
                        if (!ok) {
                            #if LOG_LEVEL >= BASIC_DEBUG
                                Serial.println("[Toe] ERROR: sensor.begin failed during reset, check wiring and I2C address");
                            #endif
                            state = ToeState::ERROR;
                            return;
                        }
                        state = ToeState::RESET_1;
                        _last_action_time = millis();
                        _unchanged_count = 0;
                    }
                    _last_range = 0.5f * _last_range + 0.5f * new_val;
                }
            }
            break;
        }
        case ToeState::RESET_1:
            // _last_range = toe_idle; // reset to idle during reset
            // TODO: Fix this reset sequence, it causes the rp2350 to reboot
            if (millis() - _last_action_time > TOE_RESET_ACTION_DELAY_MS) {
                sensor.stopRangeContinuous();
                state = ToeState::RESET_2;
                _last_action_time = millis();
            }
            break;

        case ToeState::RESET_2:
            if (millis() - _last_action_time > TOE_RESET_ACTION_DELAY_MS) {
                sensor.startRangeContinuous(10);
                _last_action_time = millis();
                state = ToeState::RESET_3;
            }
            break;
        case ToeState::RESET_3:
            if (millis() - _last_action_time > TOE_RESET_ACTION_DELAY_MS) {
                _last_action_time = millis();
                state = ToeState::OPERATING;
            }
            break;

        case ToeState::ERROR:
            #if LOG_LEVEL >= BASIC_DEBUG
                Serial.println("[Toe] ERROR: sensor in error state, cannot update");
            #endif
            break;
        default:
            #if LOG_LEVEL >= BASIC_DEBUG
                Serial.println("[Toe] ERROR: unknown state in update()");
            #endif
            break;
    }

    
    // -------------------------
    // CALIBRATION MODE -- collects samples to determine toe_idle at startup. User will copy stable reading to config.hpp TOE_IDLE_READ for future runs.
    // -------------------------
    // if (CALIBRATING_TOE)
    // {
    //     #if LOG_LEVEL >= BASIC_DEBUG
    //             Serial.printf("[Toe CAL] new_val=%.2f (last_range updated)\n", new_val);
    //         #endif
    //     if (raw == 0)
    //     {
    //         #if LOG_LEVEL >= BASIC_DEBUG
    //             Serial.println("[Toe CAL] ignoring zero range sample; sensor may not be ready yet");
    //         #endif
    //         return;
    //     }
    //     // ring buffer sum approach
    //     if (_calib_count == CALIB_SAMPLES)
    //     {
    //         _calib_sum -= _calib_buffer[_calib_index];
    //     }
    //     _calib_buffer[_calib_index] = new_val;
    //     _calib_sum += new_val;
    //     _calib_index = (_calib_index + 1) % CALIB_SAMPLES;
    //     if (_calib_count < CALIB_SAMPLES)
    //         _calib_count++;
    //     if (_calib_count == CALIB_SAMPLES)
    //     {
    //         float avg = _calib_sum / CALIB_SAMPLES;
    //         _last_range = avg;
    //         #if LOG_LEVEL >= BASIC_DEBUG
    //             Serial.printf("[Toe CAL] avg=%.2f (last_range updated)\n", avg);
    //         #endif
    //     }

    //     return;
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

uint8_t Toe::medianFilter(uint8_t sample)
{
    _median_buffer[_median_index] = sample;
    _median_index = (_median_index + 1) % 3;

    if (_median_count < 3)
        _median_count++;

    // Don't filter until buffer is full
    if (_median_count < 3)
        return sample;

    uint8_t a = _median_buffer[0];
    uint8_t b = _median_buffer[1];
    uint8_t c = _median_buffer[2];

    // Sort network
    if (a > b)
    {
        uint8_t t = a;
        a = b;
        b = t;
    }

    if (b > c)
    {
        uint8_t t = b;
        b = c;
        c = t;
    }

    if (a > b)
    {
        uint8_t t = a;
        a = b;
        b = t;
    }

    return b;
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
    return true;
    // Wire.beginTransmission(0x29); // VL6180X default address
    // return Wire.endTransmission() == 0;
}