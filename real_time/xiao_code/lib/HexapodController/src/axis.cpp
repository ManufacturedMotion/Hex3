#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include "axis.hpp"
#include "mux.hpp"

//Must call runSpeed() frequently when moving motors

Axis::Axis() : _mux(nullptr){}

void Axis::link(uint8_t pin_a, uint8_t pin_b, uint8_t encoder_ch, Mux& mux_ref) {
    _pin_a = pin_a;
    _pin_b = pin_b;
    _encoder_ch = encoder_ch;
    _mux = &mux_ref;
    _initializeAxis();
}

void Axis::link(uint8_t pin_a, uint8_t pin_b, uint8_t pin_c, uint8_t pin_d, uint8_t encoder_ch, Mux& mux_ref) {
    _pin_a = pin_a;
    _pin_b = pin_b;
    _pin_c = pin_c;
    _pin_d = pin_d;
    _encoder_ch = encoder_ch;
    _mux = &mux_ref;
    _initializeAxis();
}

void Axis::_initializeAxis() {
    pinMode(_pin_a, OUTPUT);
    pinMode(_pin_b, OUTPUT);
    if (_pin_c != 0 && _pin_d != 0) {
        _4_pin = true;
        pinMode(_pin_c, OUTPUT);
        pinMode(_pin_d, OUTPUT);
    }
    digitalWrite(_pin_a, LOW);
    digitalWrite(_pin_b, LOW);  
    if (_4_pin) {
        digitalWrite(_pin_c, LOW);
        digitalWrite(_pin_d, LOW);  
    }
} 

void Axis::initializePositionLimits(double min_pos, double max_pos) {
    _min_pos = min_pos;
    _max_pos = max_pos;
}

double Axis::getCurrentPos() {
    if (_mux == nullptr){
        return -3000;
    }    
    return _mux->readEncoder(_encoder_ch);
}

void Axis::setSpeed(double speed) {
    _speed = speed;
}

//TODO need to have some limits & logic here. do not want to always rotate shortest path, might end up breaking leg
void Axis::updatePos() { 
    float current_pos = getCurrentPos();
    float error = _target_pos - current_pos;
    if (error > 180.0) {
        error -= 360.0;
    }
    if (error < -180.0) {
        error += 360.0;
    }
    if (fabs(error) <= POS_TOLERANCE) {
        stopAxis();
        return;
    }
    Serial.print("Current Pos: ");
    Serial.print(current_pos);
    Serial.print(" Target Pos: ");  
    Serial.print(_target_pos);
    Serial.print(" Raw error: ");
    Serial.print(_target_pos - current_pos);
    Serial.print(" Wrapped error: ");
    Serial.println(error);
    Serial.print("\n");
    if (error < 0) {
        analogWrite(_pin_a, (_speed));
        digitalWrite(_pin_b, LOW);
        if (_4_pin) {
            digitalWrite(_pin_c, LOW);
            analogWrite(_pin_d, (_speed));
        }
    } 
    else {
        digitalWrite(_pin_a, LOW);
        analogWrite(_pin_b, _speed);
        if (_4_pin) {
            analogWrite(_pin_c, _speed);
            digitalWrite(_pin_d, LOW);
        }
    }
}

uint8_t Axis::moveToPos(double pos) {
    if (pos < _min_pos || pos > _max_pos)
        return 255;     //Move out of range
    if (_next_go_time > millis())
        return 254;     //Moving too quickly
    uint32_t millis_to_pos = (fabs(getCurrentPos() - pos) / _max_speed) * 1000;
    _next_go_time = millis() + millis_to_pos;
    _target_pos = _axisMap(pos); 
    return _target_pos;
}

uint8_t Axis::moveToPosAtSpeed(double pos, double target_speed) {
    if (pos < _min_pos || pos > _max_pos)
        return 255; // move out of range
    double speed = target_speed;
    uint8_t retval = 0;
    if (target_speed > _max_speed) {
        speed = _max_speed;
        retval = 1; // move speed capped
    }
    if ((getCurrentPos() - pos) <= POS_TOLERANCE) {
        _move_time = 1;
        stopAxis();
    }
    else {
        _move_time = (fabs(getCurrentPos() - pos) / speed) * 1000;
    }
    _start_rads = getCurrentPos();
    _end_rads = pos;
    _target_pos = _axisMap(pos);
    _move_progress = 0;
    _move_start_time = millis();
    _speed = speed;
    return retval;
}

double Axis::runSpeed() { //TODO - fixme
    updatePos(); return 1; //temp?

    _move_progress = (float)(millis() - _move_start_time) / ((float) _move_time);
    if (_move_progress <= 1.0) {
        double angle = _move_progress * (_end_rads - _start_rads) + _start_rads;
        return moveToPos(angle);
    }
    return 253;
}

void Axis::stopAxis() {
    digitalWrite(_pin_a, HIGH);
    digitalWrite(_pin_b, HIGH);
    if (_4_pin) {
        digitalWrite(_pin_c, HIGH);
        digitalWrite(_pin_d, HIGH);
    }
    Serial.println("Axis Stopped");
}

_Bool Axis::setMapping(double zero_pos, double map_mult, _Bool reverse_axis) {
    _zero_pos = zero_pos;
    _map_mult = map_mult;
    _reverse_axis = reverse_axis;
    //moveToPos(0);
    return true;
}

double Axis::_axisMap(double x) {
    while (x > M_PI) {
        x -= (2.0 * M_PI);
    }
    while (x < -M_PI) {
        x += (2.0 * M_PI);
    }
    if (_reverse_axis) 
        return 180 - (_radsToDegrees((x + _zero_pos) * _map_mult));
    else
        return (_radsToDegrees((x + _zero_pos) * _map_mult));
}

double Axis::_radsToDegrees(double rads) {
    return rads * 180.0 / M_PI;
}

double Axis::_degreesToRads(double degrees) {
    return degrees * M_PI / 180.0;
}

_Bool Axis::setMaxPos(double max_pos) {
    _max_pos = max_pos;
    return true;
}
_Bool Axis::setMinPos(double min_pos) {
    _min_pos = min_pos;
    return true;
}

_Bool Axis::setMaxSpeed(double max_speed) {
    _max_speed = max_speed;
    return true;
};

double Axis::getMaxSpeed() {
    return _max_speed;
};

double Axis::getMaxPos() {
    return _max_pos;
}
double Axis::getMinPos() {
    return _min_pos;
}

void Axis::detach() {
    digitalWrite(_pin_a, LOW);
    digitalWrite(_pin_b, LOW);
    if (_4_pin) {
        digitalWrite(_pin_c, LOW);
        digitalWrite(_pin_d, LOW);
    }
    Serial.println("Axis Detached");
}