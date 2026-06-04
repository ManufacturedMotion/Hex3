/**
 * @file axis.cpp
 * @brief Implementation of the Axis class for motor control and feedback
 *
 * Provides:
 * - Motor PWM control with direction support (2-pin and 4-pin configurations)
 * - Encoder position feedback with I2C multiplexer interface
 * - Cascaded PID control (position loop -> velocity loop -> torque/PWM output)
 * - Velocity and acceleration tracking with low-pass filtering
 * - Motor disturbance estimation using momentum observer (MOB)
 * - Friction model compensation for better low-speed performance
 */

#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include "axis.hpp"
#include "mux.hpp"
#include <RP2040_PWM.h>

/// PWM frequency for motor control signals
#define PWM_FREQUENCY 50000 // Hz

/**
 * @brief Map a value from one range to another
 *
 * Linear interpolation function used for duty cycle correction.
 *
 * @param x Input value to map
 * @param in_min Input range minimum
 * @param in_max Input range maximum
 * @param out_min Output range minimum
 * @param out_max Output range maximum
 * @return Mapped value constrained to output range
 */
float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
    x = constrain(x, in_min, in_max);
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Constructor - Initialize axis with null/default values
 *
 * Creates an uninitialized axis. Must call link() before use.
 */
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

/**
 * @brief Link axis to 2-pin PWM motor
 *
 * Configures a simple 2-pin motor: one pin for forward PWM, one for reverse PWM.
 * Creates RP2040_PWM instances for hardware PWM generation.
 *
 * @param pin_a PWM pin for forward direction
 * @param pin_b PWM pin for reverse direction
 * @param encoder_ch Multiplexer channel for encoder (0-7)
 * @param mux_ref Reference to the I2C multiplexer
 *
 * @note Call after constructor and before initializeAxes()
 */
void Axis::link(uint8_t pin_a, uint8_t pin_b, uint8_t encoder_ch, Mux& mux_ref) {
    _pin_a = pin_a;
    _pin_b = pin_b;
    _pwm_instances[0] = new RP2040_PWM(_pin_a, PWM_FREQUENCY, 0);
    _pwm_instances[1] = new RP2040_PWM(_pin_b, PWM_FREQUENCY, 0);
    _encoder_ch = encoder_ch;
    _mux = &mux_ref;
    _initializeAxis();
}

/**
 * @brief Link axis to 4-pin motor driver (separate enable and direction pins per motor)
 *
 * Configures a dual-motor or high-current setup with 4 PWM pins for independent
 * control of motor enable and direction. Useful for doubled torque configurations.
 *
 * @param pin_a PWM for motor 1 forward
 * @param pin_b PWM for motor 1 reverse
 * @param pin_c PWM for motor 2 forward
 * @param pin_d PWM for motor 2 reverse
 * @param encoder_ch Multiplexer channel for encoder (0-7)
 * @param mux_ref Reference to the I2C multiplexer
 *
 * @note Call after constructor and before initializeAxes()
 */
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

/**
 * @brief Initialize GPIO pins to OUTPUT and drive low
 *
 * Sets all motor control pins as outputs and ensures they start in safe (no-drive) state.
 * Automatically detects 4-pin vs 2-pin configuration.
 */
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
    
/**
 * @brief Set joint angle limits (soft limits for safety)
 *
 * Stores minimum and maximum allowable positions. Attempts to move outside
 * these limits will be rejected.
 *
 * @param min_pos Minimum angle in radians
 * @param max_pos Maximum angle in radians
 */
void Axis::initializePositionLimits(float min_pos, float max_pos) {
    _min_pos = min_pos;
    _max_pos = max_pos;
}

/**
 * @brief Get current position estimate
 *
 * Returns the most recent position reading processed by trackMotion().
 * Safe to call from control loops without I2C latency.
 *
 * @return Current position in radians
 */
float Axis::getCurrentPos() {
    return _current_pos;
}

/**
 * @brief Get current velocity estimate (filtered)
 *
 * Returns smoothed velocity calculated from position derivatives.
 * Uses low-pass filter (0.1 new + 0.9 old) to reduce noise.
 *
 * @return Velocity in rad/s
 */
float Axis::getCurrentVelocity() {
    return _current_velocity;
}

/**
 * @brief Get current acceleration estimate
 *
 * Acceleration is calculated as the rate of change of velocity.
 * Generally noisy - useful for diagnostics but not for control.
 *
 * @return Acceleration in rad/s^2
 */
float Axis::getCurrentAcceleration() {
    return _current_acceleration;
}

/**
 * @brief Private method to read current position from encoder via multiplexer
 *
 * Performs I2C read from encoder through multiplexer channel, then normalizes
 * angle to [-π, π] range and applies zero offset calibration.
 *
 * @return Position in radians, normalized to [-pi, pi], or NAN if read fails
 * @note This is a blocking call (~1-3ms) due to I2C communication
 */
float Axis::_getCurrentPos() {
    if (_mux == nullptr){
        return NAN;
    }  
    float encoder_reading = _mux->readEncoder(_encoder_ch);
    if (isnan(encoder_reading)) {
        return NAN;
    }
    encoder_reading -= _zero_pos;
    if (encoder_reading > M_PI) {
        encoder_reading -= 2.0 * M_PI;
    }
    if (encoder_reading < -M_PI) {
        encoder_reading += 2.0 * M_PI;
    }
    return encoder_reading;
}

/**
 * @brief Check if current position is within tolerance of target
 *
 * @return true if |target_pos - current_pos| <=AXIS_POSITION_TOLERANCE
 */
bool Axis::targetPosReached() {
    if (fabs(_target_pos - _current_pos) <= AXIS_POSITION_TOLERANCE) {
        return true;
    }
    return false;
}

/**
 * @brief Main tracking update loop for position, velocity, and disturbance estimation
 *
 * **Must be called regularly (1-10 ms)** Performs two independent time-triggered updates:
 *
 * **Position Phase** (interval = AXIS_POSITION_TRACK_INTERVAL_MS):
 * - Reads encoder via I2C (blocking ~1-3ms)
 * - Updates current position
 * - Estimates motor current and torque
 * - Runs momentum observer for disturbance detection
 *
 * **Velocity Phase** (interval = AXIS_VELOCITY_TRACK_INTERVAL_MS):
 * - Calculates velocity from position change
 * - Low-pass filters: v_new = 0.1*raw + 0.9*v_old
 * - Calculates acceleration as dv/dt
 * - Handles angle wrapping (shortest path)
 *
 * Separate update rates balance computational load with control responsiveness.
 */
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

/**
 * @brief Convert desired torque to PWM duty cycle
 *
 * Uses motor model: V = I*R + ω*Ke where:
 * - V = applied voltage (duty cycle × V_supply)
 * - I = motor current (τ / Kt)
 * - Ke = back EMF constant
 * - ω = current velocity
 *
 * @param torque Target torque in Nm
 * @return Duty cycle (0-100%) needed to produce that torque
 */
float Axis::_torqueToDutyCycle(float torque) {
    float target_current = torque / _torque_constant; // I = T / Kt
    float target_duty = (target_current * _resistance + _current_velocity * _back_EMF_constant) / getInputVoltage() * 100.0;
    return target_duty;
}

/**
 * @brief Estimate friction force (feedforward compensation)
 *
 * Friction is speed-dependent. Uses constant friction estimate.
 *
 * @return Estimated friction torque in units compatible with torque control
 */
float Axis::_getEstimatedFriction() {
    return _friction_constant;
}

/**
 * @brief Update internal estimate of motor current from electrical measurements
 *
 * Calculates motor current from:
 * - Applied voltage (duty cycle × V_supply)
 * - Back EMF (velocity × Ke)
 * - Motor resistance
 *
 * Updates torque estimate: τ = 0.2*I*Kt + 0.8*τ_prev (exponential filter)
 *
 * Note: Doubles current estimate if 4-pin (dual motor) configuration.
 */
void Axis::_updateMotorCurrentEstimate() {
    _estimated_current = ((getDutyCycle() / 100.0 * getInputVoltage()) - _current_velocity * _back_EMF_constant) / _resistance; 
    if (_4_pin) {
        _estimated_current *= 2; // if using 4-pin control, there are two motors so we can double the current (and thus torque) 
    }
    _estimated_torque = _estimated_torque * 0.8 + (_estimated_current * _torque_constant * 0.2) * -1.0; // very rough estimate, assumes linear relationship between duty cycle and voltage, and that torque is proportional to current
}

/**
 * @brief Momentum observer for disturbance estimation
 *
 * Implements a model-based observer to estimate external disturbance torque:
 * - Tracks momentum: p = ω*I (angular momentum)
 * - Compares measured vs estimated momentum
 * - Integrates error with gain to estimate disturbance
 * - Anti-windup limits disturbance estimate magnitude
 *
 * This is useful for detecting collisions or external forces.
 * Called by trackMotion() automatically.
 *
 * **Update Rate:** MOMENTUM_MONITOR_INTERVAL_MS
 */
void Axis::momentumMonitor() {

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

/**
 * @brief Get disturbance torque estimate from momentum observer
 *
 * Returns the current estimate of external disturbance torque. 
 * Useful for detecting collisions or external forces applied to the joint.
 *
 * @return Disturbance torque estimate in Nm
 * @note Zero if momentum observer is not active for this axis
 */
float Axis::getMOBDisturbanceTorque() {
    return _MOB_disturbance_torque;
}

/**
 * @brief Set target position with range checking
 *
 * Commands a new target position for the position control loop.
 * Enforces position limits; out-of-range commands are rejected.
 *
 * @param pos Target position in radians
 * @return 0 on success, 255 if target is outside [_min_pos, _max_pos]
 * @note Position control executes at ~20Hz via cascaded PID loops
 */
uint8_t Axis::setTargetPos(float pos) {
    if (pos < _min_pos || pos > _max_pos)
        return 255;     //Move out of range
    _target_pos = pos;
    return 0;
}

/**
 * @brief Internal method to set target velocity for the velocity control loop
 *
 * Sets the desired velocity for the inner velocity PID loop. Usually called by
 * the position control loop after calculating desired velocity from position error.
 * Can also be modified by feedforward velocity for smooth motion planning.
 *
 * @param velocity Target velocity in rad/s
 * @return Always 0 (success)
 * @note This is an internal method; use setFeedforwardVelocity() for motion planning
 */
uint8_t Axis::_setTargetVelocity(float velocity) {
    _target_velocity = velocity;
    return 0;
}

/**
 * @brief Set feedforward velocity component for smooth motion control
 *
 * Adds a velocity feedforward term that is added directly to the position loop's
 * output velocity command. Useful for trajectory tracking where desired velocity
 * is known in advance (e.g., trapezoid velocity profiles).
 *
 * Formula: velocity_command = position_PID_output + (_Kv_ff * feedforward_velocity)
 *
 * @param velocity Feedforward velocity command in rad/s
 * @return Always 0 (success)
 * @note _Kv_ff is typically set to 1.0 for near-unity scaling of feedforward term
 */
uint8_t Axis::setFeedforwardVelocity(float velocity) {
    _feedforward_velocity = velocity;
    return 0;
}

/**
 * @brief Internal method to generate PWM signals to motor driver
 *
 * Converts desired duty cycle and direction into PWM commands sent to motor driver.
 * Supports both 2-pin (forward/reverse PWM) and 4-pin (dual motor or high-current) configurations.
 *
 * **2-pin mode**: Duty cycle on active pin, 0% on inactive pin
 * - Forward: pin_a=0%, pin_b=duty_cycle
 * - Reverse: pin_a=duty_cycle, pin_b=0%
 *
 * **4-pin mode**: Both pins driven, one at 0% and one at duty_cycle
 * - Forward: pin_a=0%, pin_b=duty_cycle, pin_c=0%, pin_d=duty_cycle
 * - Reverse: pin_a=duty_cycle, pin_b=0%, pin_c=duty_cycle, pin_d=0%
 *
 * Applies axis reversal logic if _reverse_axis flag is set (inverts direction).
 * PWM frequency is fixed at PWM_FREQUENCY (50kHz).
 *
 * @param dir Desired rotation direction (true=forward, false=reverse)
 * @param duty_cycle Desired PWM duty cycle in range [0.0, 100.0] (constrained)
 * @return Always 0 (success)
 */
uint8_t Axis::_setDutyCycle(bool dir, float duty_cycle) {
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
    return 0;

}

void Axis::setControlConstants(float Kp_pos, float Kd_pos, float Kp_vel, float Ki_vel, float Kv_ff) {
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
        _setDutyCycle(0, 0.0);
        return 0;
    }
    _pid_pos->Compute();
    
    _setTargetVelocity(_pos_control + (_Kv_ff * _feedforward_velocity));
    _moveAtVelocity();
    return 0;
}

uint8_t Axis::_moveAtVelocity() {
    if (_allowed_to_move == false) {
        return 255; // move not allowed
    }
    if (isnan(_target_velocity)) {
        return 254; // no target velocity set
    }
    if (_pid_vel == nullptr) {
        Serial.println("Error: PID not initialized for Axis");
        return 254; // PID not initialized
    }
    _pid_vel->Compute();

    float control = _torqueToDutyCycle(_vel_control + _getEstimatedFriction() * (_vel_control >= 0 ? 1.0 : -1.0));
                    
    float duty_cycle = constrain(control, -100.0, 100.0);
    _setDutyCycle(duty_cycle >= 0.0, fabs(duty_cycle));
    return 0;
}

uint8_t Axis::moveToPosAtSpeed(float pos, float target_speed) {
    setFeedforwardVelocity(target_speed);
    setTargetPos(pos);
    return moveToPos();
}

void Axis::stopAxis() {
    _setDutyCycle(false, 0);
}

_Bool Axis::setMapping(float zero_pos, float map_mult, _Bool reverse_axis) {
    _zero_pos = zero_pos;
    _map_mult = map_mult;
    _reverse_axis = reverse_axis;
    return true;
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
    _input_voltage = voltage;
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