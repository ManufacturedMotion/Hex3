#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include "axis.hpp"
#include "mux.hpp"
#include <RP2040_PWM.h>

//Must call runSpeed() frequently when moving motors
#define PWM_FREQUENCY 30000 // Hz, adjust as needed

Axis::Axis() : _mux(nullptr){}

void Axis::link(uint8_t pin_a, uint8_t pin_b, uint8_t encoder_ch, Mux& mux_ref) {
    _pin_a = pin_a;
    _pin_b = pin_b;
    _pwm_instances[0] = new RP2040_PWM(_pin_a, PWM_FREQUENCY, 0);
    _pwm_instances[1] = new RP2040_PWM(_pin_b, PWM_FREQUENCY, 0);
    _encoder_ch = encoder_ch;
    _mux = &mux_ref;
    _initializeAxis();
}

void Axis::link(uint8_t pin_a, uint8_t pin_b, uint8_t pin_c, uint8_t pin_d, uint8_t encoder_ch, Mux& mux_ref) {
    _pin_a = pin_a;
    _pin_b = pin_b;
    _pin_c = pin_c;
    _pin_d = pin_d;
    _pwm_instances[0] = new RP2040_PWM(_pin_a, PWM_FREQUENCY, 0);
    _pwm_instances[1] = new RP2040_PWM(_pin_b, PWM_FREQUENCY, 0);
    _pwm_instances[2] = new RP2040_PWM(_pin_c, PWM_FREQUENCY, 0);
    _pwm_instances[3] = new RP2040_PWM(_pin_d, PWM_FREQUENCY, 0);
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
    return _current_pos;
}

double Axis::getCurrentVelocity() {
    return _current_velocity;
}

double Axis::getCurrentAcceleration() {
    return _current_acceleration;
}

double Axis::_getCurrentPos() {
    if (_mux == nullptr){
        return NAN;
    }  
    return _mux->readEncoder(_encoder_ch);
}

void Axis::trackMotion() {
    static uint32_t _last_update_time = 0;
    if (millis() - _last_update_time > AXIS_MOTION_TRACK_INTERVAL_MS) {
        uint32_t delta_time = millis() - _last_update_time;
        uint32_t temp_last_update_time = millis();
        double current_pos = _getCurrentPos();
        if (isnan(current_pos)) {
            return;
        }
        _last_update_time = temp_last_update_time; //avoid time skew due to I2C delay
        _last_position = _current_pos;
        _last_velocity = _current_velocity;
        _current_pos = current_pos;
        double distance_traversed = _current_pos - _last_position;
        distance_traversed += (distance_traversed > M_PI) ? -2 * M_PI : (distance_traversed < -M_PI) ? 2 * M_PI : 0; // Assume shortest path
        _current_velocity = (_current_pos - _last_position) / (delta_time / 1000.0); //rad/s
        _current_acceleration = (_current_velocity - _last_velocity) / (delta_time / 1000.0); //rad/s^2
        
    }
}

uint8_t Axis::setTargetPos(double pos) {
    if (pos < _min_pos || pos > _max_pos)
        return 255;     //Move out of range
    _target_pos = pos;
    return 0;
}


void Axis::setSpeed(double speed) {
    _speed = speed;
}

//TODO need to have some limits & logic here. do not want to always rotate shortest path, might end up breaking leg
void Axis::updatePos() { 
    float current_pos = getCurrentPos();
    // float error = _target_pos - current_pos;
    // if (error > M_PI_2) {
    //     error -= M_2_PI;
    // }
    // if (error < -M_PI_2) {
    //     error += M_2_PI;
    // }
    // if (fabs(error) <= POS_TOLERANCE) {
    //     stopAxis();
    //     return;
    // }
    // if (error < 0) {
    //     setDutyCycle(false, _speed);
    // } 
    // else {
    //     setDutyCycle(true, _speed);
    // }
}

uint8_t Axis::setDutyCycle(bool dir, float duty_cycle) {
    duty_cycle = constrain(duty_cycle, 0.0, 100.0);
    if (dir) {
        _pwm_instances[0]->setPWM(_pin_a, PWM_FREQUENCY, 0.0);
        _pwm_instances[1]->setPWM(_pin_b, PWM_FREQUENCY, duty_cycle);
        if (_4_pin) {
            _pwm_instances[2]->setPWM(_pin_c, PWM_FREQUENCY, 0.0);
            _pwm_instances[3]->setPWM(_pin_d, PWM_FREQUENCY, duty_cycle);
        }
    } 
    else {
        _pwm_instances[0]->setPWM(_pin_a, PWM_FREQUENCY, duty_cycle);
        _pwm_instances[1]->setPWM(_pin_b, PWM_FREQUENCY, 0.0);
        if (_4_pin) {
            _pwm_instances[2]->setPWM(_pin_c, PWM_FREQUENCY, duty_cycle);
            _pwm_instances[3]->setPWM(_pin_d, PWM_FREQUENCY, 0.0);
        }
    }
    return 0;
}

void Axis::setPIDConstants(double Kp, double Ki, double Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    if (_pid != nullptr) {
        delete _pid;
    }
    _pid = new PID(&_current_pos, &_control, &_target_pos, _Kp, _Ki, _Kd, DIRECT);
    _pid->SetOutputLimits(-100, 100); 
    Serial.printf("Axis PID set: Kp=%f, Ki=%f, Kd=%f\n", Kp, Ki, Kd);
    _pid->SetMode(AUTOMATIC);
}


void Axis::allowMotion(bool allowed) {
    _allowed_to_move = allowed;
    if (!allowed) {
        stopAxis();
    }
}

uint8_t Axis::moveToPos() {
    if (_allowed_to_move == false) {
        return 255; // move not allowed
    }
    if (isnan(_target_pos)) {
        return 254; // no target position set
    }
    if (_pid == nullptr) {
        Serial.println("Error: PID not initialized for Axis");
        return 254; // PID not initialized
    }

    _pid->Compute();
    //notes for future; assume max distance is 2 rads; assume kp = .5, ki = 0, kd = 0.05; worst case we get control output of about 3.65
    //to get 100% duty cycle at max control output, need to scale by about 27.5 (100 / 3.65). 3.65 is also how I decided to set min/max for PID compute to +/- 4
    //PID library is not doing any target position wrapping, it always goes the long way around. We can assume the control speed is correct and 
    //determine direction on our own
    float scaled_duty_cycle = constrain(_control * 27.5, -100, 100.0);
    float min_duty = 55.0; //minimum duty cycle to overcome motor deadzone from standstill. Might need to bump this up when we have a load on the motors...
    if (scaled_duty_cycle != 0.0 && fabs(scaled_duty_cycle) < min_duty) {
        scaled_duty_cycle = (scaled_duty_cycle > 0) ? min_duty : -min_duty;
    }

    float error = _target_pos - _current_pos;
    float accepted_error = 0.01745; //about 1 degree in radians
    if (fabs(error) <= accepted_error) {
        scaled_duty_cycle = 0;
    }

    static uint32_t last_print_time = 0;
    if (millis() - last_print_time > 1000) {
        last_print_time = millis();
        Serial.printf("raw control is %f\n", _control);
        Serial.printf("PID Control: %f\n", scaled_duty_cycle);
    }
    setDutyCycle(scaled_duty_cycle >= 0.0, fabs(scaled_duty_cycle)); //TODO - scale control to duty cycle, commented out for safety during testing
    return 0;
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
    setDutyCycle(false, 0);
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