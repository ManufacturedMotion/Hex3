#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include "axis.hpp"
#include "mux.hpp"
#include <RP2040_PWM.h>

//Must call runSpeed() frequently when moving motors
#define PWM_FREQUENCY 50000 // Hz, adjust as needed

float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
    x = constrain(x, in_min, in_max);
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Axis::Axis() :
    _mux(nullptr),
    _pin_a(0),
    _pin_b(0),
    _pin_c(0),
    _pin_d(0),
    _4_pin(false),
    _pid_pos(nullptr),
    _pid_vel(nullptr),
    _allowed_to_move(true)
{}

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
    

void Axis::initializePositionLimits(float min_pos, float max_pos) {
    _min_pos = min_pos;
    _max_pos = max_pos;
}

float Axis::getCurrentPos() {
    return _current_pos;
}

float Axis::getCurrentVelocity() {
    return _current_velocity;
}

float Axis::getCurrentAcceleration() {
    return _current_acceleration;
}

float Axis::_getCurrentPos() {
    if (_mux == nullptr){
        return NAN;
    }  
    float encoder_reading = _mux->readEncoder(_encoder_ch);
    encoder_reading -= _zero_pos;
    if (encoder_reading > M_PI) {
        encoder_reading -= 2.0 * M_PI;
    }
    if (encoder_reading < -M_PI) {
        encoder_reading += 2.0 * M_PI;
    }
    return encoder_reading;
}

bool Axis::targetPosReached() {
    if (fabs(_target_pos - _current_pos) <= AXIS_POSITION_TOLERANCE) {
        return true;
    }
    return false;
}

void Axis::trackMotion() {
    
    if (millis() - _last_pos_update_time > AXIS_POSITION_TRACK_INTERVAL_MS) {
        uint32_t delta_time = millis() - _last_pos_update_time;
        uint32_t temp_last_update_time = millis();
        float current_pos = _getCurrentPos();
        if (isnan(current_pos)) {
            return;
        }
        _last_pos_update_time = temp_last_update_time; //avoid time skew due to I2C delay
        _current_pos = current_pos;
        _updateMotorCurrentEstimate();
        momentumMonitor();
    }
    if (millis() - _last_vel_update_time > AXIS_VELOCITY_TRACK_INTERVAL_MS) {
        uint32_t delta_time = millis() - _last_vel_update_time;
        _last_velocity = getCurrentVelocity();

        float distance_traversed = getCurrentPos() - _last_position;
        distance_traversed += (distance_traversed > M_PI) ? -2 * M_PI : (distance_traversed < -M_PI) ? 2 * M_PI : 0; // Assume shortest path
        _current_velocity = distance_traversed / (static_cast<float>(delta_time) / 1000.0); //rad/s
        _current_velocity = (0.1 * _current_velocity) + (0.9 * _last_velocity); // low-pass filter to reduce noise
        _current_acceleration = (getCurrentVelocity() - _last_velocity) / (static_cast<float>(delta_time) / 1000.0); //rad/s^2
        _last_position = getCurrentPos();
        _last_vel_update_time = millis();
       
    }
}

float Axis::_torqueToDutyCycle(float torque) {
    float target_current = torque / _torque_constant; // I = T / Kt
    
    // Serial.printf("Torque: %f Nm, Target Current: %f A, Duty Cycle: %f\n", torque, target_current, (target_current * _resistance + _current_velocity * _back_EMF_constant) / getInputVoltage() * 100.0);
    // Serial.printf("Voltage Drop: %f V, Back EMF: %f V, Input Voltage: %f V\n", target_current * _resistance, _current_velocity * _back_EMF_constant, getInputVoltage());
    float target_duty = (target_current * _resistance + _current_velocity * _back_EMF_constant) / getInputVoltage() * 100.0;
     // Add minimum duty to overcome motor deadzone, in the correct direction
    return target_duty;
    // if (unmapped > 0) {
    //     return unmappe
    //     return float_map(unmapped, 0.0, 100.0, _min_duty, 100.0);
    // }
    // else {
    //     return float_map(unmapped, -100.0, 0.0, -100.0, -_min_duty);
    // }
}

void Axis::_updateMotorCurrentEstimate() {
    _estimated_current = ((getDutyCycle() / 100.0 * getInputVoltage()) - _current_velocity * _back_EMF_constant) / _resistance; 
    if (_4_pin) {
        _estimated_current *= 2; // if using 4-pin control, there are two motors so we can double the current (and thus torque) 
    }
    _estimated_torque = _estimated_torque * 0.8 + (_estimated_current * _torque_constant * 0.2) * -1.0; // very rough estimate, assumes linear relationship between duty cycle and voltage, and that torque is proportional to current
}

void Axis::momentumMonitor() {
    // Momentum monitoring is currently only used for the 4-pin leg axis with encoder channel 6.
    if (_encoder_ch != 6) {
        return;
    }

    uint32_t now = millis();

    if (_last_momentum_monitor_update_time == 0) {
        // First execution: initialize observer state to measured momentum and reset disturbance estimate
        _momentum_hat = _current_velocity * _total_inertia;
        _MOB_disturbance_torque = 0.0f;
        _last_momentum_monitor_update_time = now;
        return;
    }

    uint32_t elapsed_ms = now - _last_momentum_monitor_update_time;
    if (elapsed_ms < MOMENTUM_MONITOR_INTERVAL_MS) {
        return;
    }

    // Clip dt to a safe range to prevent numerical issues after long pauses or scheduling jitter
    float dt = (elapsed_ms / 1000.0f);
    if (dt <= 0.0f) {
        _last_momentum_monitor_update_time = now;
        return;
    }
    if (dt > 0.1f) {
        dt = 0.1f;
    }

    _last_momentum_monitor_update_time = now;

    float tau_motor = getEstimatedTorque();
    float momentum = _current_velocity * _total_inertia;
    if (abs(momentum) < 0.001f) {
        // If momentum is very low, the estimate is likely dominated by noise, so skip the update
        return;
    }
    float error = momentum - _momentum_hat;

    // disturbance observer (integral form)
    _MOB_disturbance_torque += _momentum_monitor_gain * error * dt;

    // anti-windup for disturbance torque
    const float kMaxDisturbanceTorque = 30.0f;  // Nm (tune as needed for system)
    _MOB_disturbance_torque = constrain(_MOB_disturbance_torque, -kMaxDisturbanceTorque, kMaxDisturbanceTorque);

    // update internal momentum model
    _momentum_hat += (tau_motor + _MOB_disturbance_torque) * dt;

    // keep momentum estimate anchored to measurement to avoid long-term drift
    const float kMaxMomentumError = 0.5f * _total_inertia; // rad/s * kg*m^2, adjust as needed
    if (fabs(error) > kMaxMomentumError) {
        _momentum_hat = momentum;
    }
}

float Axis::getMOBDisturbanceTorque() {
    return _MOB_disturbance_torque;
}

void Axis::disturanceMonitor() {

}

uint8_t Axis::setTargetPos(float pos) {
    if (pos < _min_pos || pos > _max_pos)
        return 255;     //Move out of range
    _target_pos = pos;
    return 0;
}

uint8_t Axis::setTargetVelocity(float velocity) {
    _target_velocity = velocity;
    return 0;
}

uint8_t Axis::setFeedforwardVelocity(float velocity) {
    _feedforward_velocity = velocity;
    return 0;
}

void Axis::setSpeed(float speed) {
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
    if (_encoder_ch == 6) {
        float max_duty_cycle = 100.0;
        _duty_cycle = constrain(duty_cycle, 0.0, 100.0);
        _dir = dir;
        
        if (_reverse_axis) {
            _dir = !_dir;
        }
        if (_dir) {
            _pwm_instances[0]->setPWM(_pin_a, PWM_FREQUENCY, 0.0);
            _pwm_instances[1]->setPWM(_pin_b, PWM_FREQUENCY, _duty_cycle);
            if (_4_pin) {
                _pwm_instances[2]->setPWM(_pin_c, PWM_FREQUENCY, 0.0);
                _pwm_instances[3]->setPWM(_pin_d, PWM_FREQUENCY, _duty_cycle);
            }
        } 
        else {
            _pwm_instances[0]->setPWM(_pin_a, PWM_FREQUENCY, _duty_cycle);
            _pwm_instances[1]->setPWM(_pin_b, PWM_FREQUENCY, 0.0);
            if (_4_pin) {
                _pwm_instances[2]->setPWM(_pin_c, PWM_FREQUENCY, _duty_cycle);
                _pwm_instances[3]->setPWM(_pin_d, PWM_FREQUENCY, 0.0);
            }
        }
    }
    return 0;

}

void Axis::setControlConstants(float tuning_voltage, float Kp_pos, float Kd_pos, float Kp_vel, float Ki_vel, float Kv_ff) {
    _tuning_voltage = tuning_voltage;
    if (_input_voltage < 0.0) {
        _input_voltage = tuning_voltage; // default to tuning voltage if not set explicitly
    }
    _Kp_pos = Kp_pos;
    _Ki_pos = 0.0; // not currently using integral control for position, as it can lead to instability and overshoot. may add back later with some anti-windup logic
    _Kd_pos = Kd_pos;
    _Kp_vel = Kp_vel;
    _Ki_vel = Ki_vel;
    _Kd_vel = 0.0; // not currently using derivative control for velocity, as it can amplify noise. may add back later with some filtering
    _Kv_ff = Kv_ff;
    if (_pid_pos != nullptr) {
        delete _pid_pos;
    }
    _pid_pos = new PID(&_current_pos, &_pos_control, &_target_pos, _Kp_pos, _Ki_pos, _Kd_pos, DIRECT);
    _pid_pos->SetOutputLimits(-_max_speed, _max_speed); // Output is target velocity in rad/s
    _pid_pos->SetSampleTime(1);
    Serial.printf("Axis Position PID set: Kp=%f, Ki=%f, Kd=%f\n", _Kp_pos, _Ki_pos, _Kd_pos);
    _pid_pos->SetMode(AUTOMATIC);
    
    if (_pid_vel != nullptr) {
        delete _pid_vel;
    }
    _pid_vel = new PID(&_current_velocity, &_vel_control, &_target_velocity, _Kp_vel, _Ki_vel, _Kd_vel, DIRECT);
    _pid_vel->SetOutputLimits(-20.0, 20.0); // Output is target torque in Nm TODO make configurable and configure to the real torque limits of the system
    _pid_vel->SetSampleTime(AXIS_VELOCITY_TRACK_INTERVAL_MS);
    Serial.printf("Axis Velocity PID set: Kp=%f, Ki=%f, Kd=%f\n", _Kp_vel, _Ki_vel, _Kd_vel);
    _pid_vel->SetMode(AUTOMATIC);
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
    if (_pid_pos == nullptr) {
        Serial.println("Error: PID not initialized for Axis");
        return 254; // PID not initialized
    }

    float error = _target_pos - _current_pos;
    float accepted_error = AXIS_POSITION_TOLERANCE;
    if (fabs(error) <= accepted_error) {
        setDutyCycle(0, 0.0);
        return 0;
    }
    _pid_pos->Compute();
    // float control = _pos_control;
    // if (fabs(_target_velocity) > 0.001) {
    //     control += (_Kv_ff * _target_velocity);
    // }
    // if (fabs(_target_acceleration) > 0.001) {
    //     control += (_Ka_ff * _target_acceleration);
    // }
    // float scaled_duty_cycle = constrain(control, -100, 100.0);
    
    
    // if (scaled_duty_cycle > 0.0) {
    //     scaled_duty_cycle = float_map(scaled_duty_cycle, 0.0, 100.0, _min_duty, 100.0);
    // }
    // else {
    //     scaled_duty_cycle = float_map(scaled_duty_cycle, -100.0, 0.0, -100.0, -_min_duty);
    // }
    // setDutyCycle(scaled_duty_cycle >= 0.0, fabs(scaled_duty_cycle));
    setTargetVelocity(_pos_control + (_Kv_ff * _feedforward_velocity));
    moveAtVelocity();
    return 0;
}

uint8_t Axis::moveAtVelocity() {
    if (_encoder_ch != 6) {
        return 0;
    }
    if (_allowed_to_move == false) {
        return 255; // move not allowed
    }
    if (isnan(_target_velocity)) {
        return 254; // no target velocity set
    }
    if (_pid_pos == nullptr) {
        Serial.println("Error: PID not initialized for Axis");
        return 254; // PID not initialized
    }
    _pid_vel->Compute();
    
    
    float Ks = _min_duty;
    float Kc = 45.0;
    float Kv_friction = 1.0; // How much additional torque is needed per unit of velocity to overcome friction. Adjust as needed for system
    float vs = 0.5; // Stribeck velocity, adjust as needed for system

    float sign = (_current_velocity >= 0) ? -1.0 : 1.0;

    float stribeck = (Ks - Kc) * exp(-pow(fabs(_current_velocity)/vs, 2.0));

    float control = _torqueToDutyCycle(_vel_control);
                    // + sign * (Kc + stribeck)
                    // + _target_velocity * Kv_friction;

    // Serial.printf("Torque Command: %f, Velocity Control: vel_control=%f, feedforward=%f, stribeck=%f, friction=%f, total_control=%f\n", _torqueToDutyCycle(_vel_control), (_Kv_ff * _feedforward_velocity), sign * stribeck, sign * Kv_friction * _target_velocity, control);

    float duty_cycle = constrain(control, -100.0, 100.0);
    setDutyCycle(duty_cycle >= 0.0, fabs(duty_cycle));
    return 0;
}

uint8_t Axis::moveToPosAtSpeed(float pos, float target_speed) {
    if (_allowed_to_move == false) {
        return 255; // move not allowed
    }
    if (isnan(_target_pos)) {
        return 254; // no target position set
    }
    if (_pid_pos == nullptr) {
        Serial.println("Error: PID not initialized for Axis");
        return 254; // PID not initialized
    }
    _pid_pos->Compute();
    float error = _target_pos - _current_pos;
    float accepted_error = AXIS_POSITION_TOLERANCE;
    if (fabs(error) <= accepted_error) {
        setDutyCycle(0, 0.0);
        return 0;
    }

    return 1;
}

float Axis::runSpeed() { //TODO - fixme
    updatePos(); return 1; //temp?

    _move_progress = (float)(millis() - _move_start_time) / ((float) _move_time);
    if (_move_progress <= 1.0) {
        float angle = _move_progress * (_end_rads - _start_rads) + _start_rads;
        return moveToPos(angle);
    }
    return 253;
}

void Axis::stopAxis() {
    setDutyCycle(false, 0);
}

_Bool Axis::setMapping(float zero_pos, float map_mult, _Bool reverse_axis) {
    _zero_pos = zero_pos;
    _map_mult = map_mult;
    _reverse_axis = reverse_axis;
    return true;
}

float Axis::_axisMap(float x) {
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

float Axis::_radsToDegrees(float rads) {
    return rads * 180.0 / M_PI;
}

float Axis::_degreesToRads(float degrees) {
    return degrees * M_PI / 180.0;
}

_Bool Axis::setMaxPos(float max_pos) {
    _max_pos = max_pos;
    return true;
}
_Bool Axis::setMinPos(float min_pos) {
    _min_pos = min_pos;
    return true;
}

_Bool Axis::setMaxSpeed(float max_speed) {
    _max_speed = max_speed;
    return true;
};

void Axis::setInputVoltage(float voltage) {
    // if (abs(_input_voltage - voltage) > 0.1) { // Only update if the voltage has changed significantly to avoid noise causing fluctuations in current estimation
        _input_voltage = voltage;
        // _pid_vel->SetTunings(
        //     _Kp_vel * _tuning_voltage / _input_voltage, 
        //     _Ki_vel * _tuning_voltage / _input_voltage,
        //     _Kd_vel * _tuning_voltage / _input_voltage
        // ); // Update PID tunings to account for change in voltage affecting system response
    // }
}

float Axis::getInputVoltage() {
    return _input_voltage;
}

float Axis::getEstimatedTorque() {
    return _estimated_torque;
}

float Axis::getDutyCycle() {
    return _dir ? _duty_cycle : -_duty_cycle;
}

float Axis::getCorrectedDutyCycle() {
    float real_duty = getDutyCycle();
    if (real_duty > 0.01) {
        return float_map(real_duty, _min_duty, 100.0, 0.0, 100.0);
    }
    else if (real_duty < -0.01) {
        return float_map(real_duty, -100.0, -_min_duty, -100.0, 0.0);
    }
    else {
        return 0.0;
    }
}

float Axis::getMaxSpeed() {
    return _max_speed;
};

float Axis::getMaxPos() {
    return _max_pos;
}
float Axis::getMinPos() {
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