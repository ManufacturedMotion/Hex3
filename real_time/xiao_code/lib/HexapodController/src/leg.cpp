/**
 * @file leg.cpp
 * @brief Implementation of Leg class for hexapod robot control
 *
 * Provides complete leg control including:
 * - Inverse and forward kinematics calculations
 * - Motion control with velocity profiles (acceleration/deceleration)
 * - Real-time telemetry logging to serial
 * - Position and velocity tracking
 */

#include "leg.hpp"
#include "axis.hpp"
#include "config.hpp"
#include "can.hpp"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "three_by_matrices.hpp"
#include <Arduino.h>
#include "log_levels.hpp"
#include "mux.hpp"

// Configuration tables loaded from config.hpp
double zero_points[NUM_LEGS][NUM_AXES_PER_LEG] = ZERO_POINTS;
double min_pos[NUM_LEGS][NUM_AXES_PER_LEG] = MIN_POS;
double max_pos[NUM_LEGS][NUM_AXES_PER_LEG] = MAX_POS;
double scale_fact[NUM_LEGS][NUM_AXES_PER_LEG] = SCALE_FACT;
_Bool reverse_axis[NUM_LEGS][NUM_AXES_PER_LEG] = REVERSE_AXIS;

/// Local enumeration for Cartesian dimensions
enum Dimension { X = 0, Y = 1, Z = 2};

/**
 * @brief Constructor - Initialize leg to invalid state
 */
Leg::Leg() {
    _leg_number = 0;
    can = nullptr;
}
/**
 * @brief Initialize hardware - GPIO, multiplexer, and axis links
 *
 * Sets up the multiplexer and links each axis to its servo pins.
 * Pulse lines: [D11, D18, D16] are the PWM outputs
 * Direction lines: [D12, D2, D15] control motor direction
 * Also enables analog input for toe pressure sensor
 */
void Leg::begin(){
    mux.begin();
    axes[0].link(D8, D10, 5, mux);
    axes[1].link(D11, D12, D15, D16, 6, mux);
    axes[2].link(D17, D18, 7, mux);
    pinMode(TOE_PIN, INPUT); 
    //if (can){
    //    can->begin();
    //}
}

/**
 * @brief Initialize axes with calibration data for this leg
 *
 * Loads joint limits and sensor calibration from config tables based on leg number.
 *
 * @param leg_number The leg identifier (0-5 for hexapod, 0-2 for tripod)
 */
void Leg::initializeAxes(uint8_t leg_number) {
    _leg_number = leg_number;
    if (can == nullptr) {
        can = new Can(1, 0, CanBitRate::BR_500k, leg_number, this);
    }
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        axes[i].initializePositionLimits(min_pos[_leg_number][i], max_pos[_leg_number][i]);
        axes[i].setMapping(zero_points[_leg_number][i], scale_fact[_leg_number][i], reverse_axis[_leg_number][i]);
    }
}

/**
 * @brief Update motion tracking - position, velocity, and acceleration estimates
 *
 * Called regularly to:
 * - Track axis motion through their trackMotion() methods
 * - Update Cartesian position using forward kinematics
 * - Calculate velocity and acceleration from position changes
 *
 * Uses separate update intervals for position (fast) and velocity (slower)
 * to reduce computational load.
 */
void Leg::_trackMotion() {
    for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
        axes[j].trackMotion();
    }
    
    // Update Cartesian position periodically using forward kinematics
    if (millis() - _last_pos_update_time > LEG_POSITION_TRACK_INTERVAL_MS) {
        _last_pos_update_time = millis();
        // Calculate XYZ from joint angles
        _forwardKinematics(axes[0].getCurrentPos(), axes[1].getCurrentPos(), axes[2].getCurrentPos(), 
                          _current_pos[X], _current_pos[Y], _current_pos[Z]);
    }
    
    // Update velocity and acceleration (less frequently to improve stability)
    if (millis() - _last_velocity_update_time > LEG_VELOCITY_TRACK_INTERVAL_MS) {
        uint32_t delta_time = millis() - _last_velocity_update_time;
        _last_velocity_update_time = millis();
        
        for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
            _last_velocity[j] = _current_velocity[j];
            double distance_traversed = _current_pos[j] - _last_pos[j];
            _current_velocity[j] = distance_traversed / (static_cast<double>(delta_time) / 1000.0); // units/s
            _current_acceleration[j] = (_current_velocity[j] - _last_velocity[j]) / (static_cast<double>(delta_time) / 1000.0); // units/s^2
            _last_pos[j] = _current_pos[j];
        }
    }
}

/**
 * @brief Main control loop - Execute PID, update motors, log telemetry, track motion
 *
 * Call this regularly (every 1-2ms) from your main loop. Performs:
 * - PID control for each axis to reach target position
 * - PWM signal generation through moveToPos()
 * - Voltage feedback to motor controllers
 * - JSON telemetry logging every 10ms
 * - Motion tracking (position/velocity updates)
 *
 * The telemetry format is JSON with axis position, velocity, acceleration, duty cycle, and estimated torque.
 */
void Leg::runSpeed() {
    static uint32_t last_print_time = 0;
    
    // Log telemetry every 10ms
    if (millis() - last_print_time > 10) {
        last_print_time = millis();
#if TELEMETRY_LOGGING_SPACE == TELEMETRY_LOGGING_SPACE_CARTESIAN
        Serial.printf("{\"Cartesian\": {\"pos\": [%f, %f, %f], \"vel\": [%f, %f, %f], \"acc\": [%f, %f, %f], \"duty\": [%f, %f, %f]}, \"voltage\": %f}\n",
            _current_pos[X], _current_pos[Y], _current_pos[Z],
            _current_velocity[X], _current_velocity[Y], _current_velocity[Z],
            _current_acceleration[X], _current_acceleration[Y], _current_acceleration[Z],
            axes[0].getDutyCycle(), axes[1].getDutyCycle(), axes[2].getDutyCycle(),
            voltage_sensor.filteredRead());
#elif TELEMETRY_LOGGING_SPACE == TELEMETRY_LOGGING_SPACE_JOINT
        Serial.printf("{\"Joint\": {\"pos\": [%f, %f, %f], \"vel\": [%f, %f, %f], \"acc\": [%f, %f, %f], \"duty\": [%f, %f, %f]}, \"voltage\": %f}\n",
            axes[0].getCurrentPos(), axes[1].getCurrentPos(), axes[2].getCurrentPos(),
            axes[0].getCurrentVelocity(), axes[1].getCurrentVelocity(), axes[2].getCurrentVelocity(),
            axes[0].getCurrentAcceleration(), axes[1].getCurrentAcceleration(), axes[2].getCurrentAcceleration(),
            axes[0].getDutyCycle(), axes[1].getDutyCycle(), axes[2].getDutyCycle(),
            voltage_sensor.filteredRead());
#endif
    }
    
    // Execute PID control and motor commands for all axes
    for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
        axes[j].moveToPos();
        axes[j].setInputVoltage(voltage_sensor.filteredRead());
    }
    
    // Update motion tracking (kinematics, velocity estimates)
    _trackMotion();
}

/**
 * @brief Configure PID and feedforward parameters for position control on an axis
 *
 * @param axis_number  Axis to configure (0-2)
 * @param tuning_voltage Reference voltage for tuning (typically supply voltage)
 * @param Kp_pos       Position loop proportional gain
 * @param Kd_pos       Position loop derivative gain
 * @param Kp_vel       Velocity loop proportional gain
 * @param Ki_vel       Velocity loop integral gain
 * @param Kv_ff        Velocity feedforward gain
 */
void Leg::setAxisControlConstants(uint8_t axis_number, double Kp_pos, double Kd_pos, double Kp_vel, double Ki_vel, double Kv_ff) {
    axes[axis_number].setControlConstants(Kp_pos, Kd_pos, Kp_vel, Ki_vel, Kv_ff);
}

/**
 * @brief Calculate Cartesian position (XYZ) from joint angles using forward kinematics
 *
 * Implements 3-joint forward kinematics for the leg mechanism.
 * Uses DH parameters: _length0 (base), _length1, and _length2.
 *
 * @param theta0 Joint 0 angle (yaw/rotation in horizontal plane) [radians]
 * @param theta1 Joint 1 angle (pitch/elevation) [radians]
 * @param theta2 Joint 2 angle (pitch/reach extension) [radians]
 * @param[out] x Cartesian X position [mm]
 * @param[out] y Cartesian Y position [mm]
 * @param[out] z Cartesian Z position (height) [mm]
 * @return Always true (no error checking in current implementation)
 */
_Bool Leg::_forwardKinematics(double theta0, double theta1, double theta2, double& x, double& y, double& z) {
    // Calculate distance in XY plane from vertical
    double planar_distance = _length1 * cos(theta1) + _length2 * cos(theta1 + theta2) + _length0;
    
    // Project planar distance to X,Y based on yaw angle
    x = planar_distance * sin(theta0);
    y = planar_distance * cos(theta0);
    
    // Calculate height (Z is measured downward from the hip)
    z = -_length1 * sin(theta1) - _length2 * sin(theta1 + theta2);
    return true;
}

/**
 * @brief Calculate joint angles from Cartesian position (inverse kinematics)
 *
 * Solves for joint angles (theta0, theta1, theta2) given a target XYZ position.
 * Updates _next_angles[] with the solution if valid.
 * Uses analytic solution with special handling for edge cases.
 *
 * @param x Target X position [mm]
 * @param y Target Y position [mm]
 * @param z Target Z position [mm]
 * @return true if solution found and valid, false if out of reach or NaN
 *
 * @note On successful solution, _next_angles will contain the target joint angles.
 *       If the position is out of joint limits, the computed angles are still stored.
 */
_Bool Leg::_inverseKinematics(double x, double y, double z) {

    // Validate coordinates are reachable
    if (!_checkSafeCoords(x, y, z))
        return false; 
        
    double potential_results[3];
    
    // Calculate theta0 (yaw angle)
    // Special handling for coordinates near y=0 to avoid division issues
    if (fabs(y < 0.1)) {
        if (x > 0.1) {
            potential_results[0] = M_PI / 2.0;
        }
        else if (x < -0.1) {
            potential_results[0] = -M_PI / 2.0;
        }
        else {
            potential_results[0] = 0;
        }
    }  
    else {
        potential_results[0] = atan2(x, y);
    }

    // Calculate effective planar distance
    double y_virtual_planar = sqrt(y * y + x * x) - _length0;
    double planar_distance = sqrt(y_virtual_planar * y_virtual_planar + z * z);
    
    // Use law of cosines to calculate theta2 (elbow angle)
    double theta2_tool = (planar_distance*planar_distance - _length1*_length1 - _length2*_length2)
                        / (2 * _length1 * _length2);

    potential_results[2] = atan2(sqrt(1 - (theta2_tool*theta2_tool)), theta2_tool);
    
    // Calculate theta1 (shoulder angle)
    double theta1_tool0 = atan2(z, y_virtual_planar);
    double theta1_tool1 = atan2(_length2 * sin(potential_results[2]), _length1 + _length2 * cos(potential_results[2]));
    potential_results[1] = theta1_tool0 - theta1_tool1;
    
    // Adjust angles for negative Z (target below the shoulder plane)
    if (z < 0) {
        potential_results[2] = -potential_results[2];
        potential_results[1] = theta1_tool0 + theta1_tool1;
    }
    
    // Validate solution: check for NaN and joint limits
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        if (potential_results[i] != potential_results[i]) { // NaN check
            return false;
        }   
        
        #ifdef LEG_DEBUG
            // Log violations of joint limits (but still accept the solution)
            if (potential_results[i] < min_pos[_leg_number][i] || potential_results[i] > max_pos[_leg_number][i]) {
                Serial.printf("Potential result %d: %f is ", i, potential_results[i]);
                Serial.println("OUT OF BOUNDS");
            }
            else {
                _next_angles[i] = potential_results[i];
            }
        #endif
    }
    
    return true;
}

/**
 * @brief Validate that Cartesian coordinates are safe for the leg
 *
 * Currently always returns true - placeholder for collision checking
 * with body or other legs. Can be extended in future.
 *
 * @param x Cartesian X coordinate [mm]
 * @param y Cartesian Y coordinate [mm]
 * @param z Cartesian Z coordinate [mm]
 * @return true if coordinates are safe, false if collision detected
 */
_Bool Leg::_checkSafeCoords(double x, double y, double z) {
    // TODO: Implement collision checking with hexapod body
    return true;
}

/**
 * @brief Move leg to target Cartesian position immediately (no acceleration profile)
 *
 * Calculates target joint angles using inverse kinematics and sets them as
 * immediate targets for the PID controllers. The leg will accelerate naturally
 * based on motor torque and dynamics.
 *
 * @param x Target X position [mm]
 * @param y Target Y position [mm]
 * @param z Target Z position [mm]
 * @return true if target is reachable, false if IK fails (out of reach)
 *
 * @note Does not update _moving_flag. Call this repeatedly if continuous motion is needed.
 *       Use linearMoveSetup() for coordinated motion with velocity control.
 */
_Bool Leg::rapidMove(double x,  double y, double z) {
    if (_inverseKinematics(x, y, z)) {
        _moveAxes();
        _current_cartesian[0] = x;
        _current_cartesian[1] = y;
        _current_cartesian[2] = z;
        return true;
    }
    return false;
}

_Bool Leg::rapidMove(ThreeByOne target_pos) {
    return rapidMove(target_pos.values[0], target_pos.values[1], target_pos.values[2]);
}

/**
 * @brief Execute one iteration of a linear move with acceleration profile
 *
 * Must be called regularly (every 1-5ms) after linearMoveSetup().
 * Implements a three-phase motion profile:
 *   1. ACCELERATING: Linear ramp up to target speed
 *   2. CRUISING: Constant velocity motion
 *   3. DECELERATING: Linear ramp down to zero
 *
 * Each phase uses trapezoidal velocity profile with MAX_LINEAR_ACCELERATION.
 *
 * @return 1 if move is active/progressing, 0 if move not active or move interval not yet elapsed
 *
 * @note Updates position targets continuously using inverse kinematics.
 *       Sets feedforward velocity on axes for smooth control.
 *       Call until return value is 0 to complete the move.
 */
uint8_t Leg::linearMovePerform() {
    uint32_t delta = millis() - _last_linear_move_time;
    
    if (delta >= LINEAR_MOVE_INTERVAL_MS) {
        _last_linear_move_time = millis();
        
        if (_move_stage == move_stage::ACCELERATING || _move_stage == move_stage::CRUISING || _move_stage == move_stage::DECELERATING) {
            double move_progress;
            
            // Calculate progress through current phase (0.0 to 1.0)
            switch(_move_stage) {
                case ACCELERATING:
                    move_progress = (float)(millis() - _move_start_time) / ((float) _accel_time);
                    break;
                case CRUISING:
                    move_progress = (float)(millis() - _move_start_time - _accel_time) / ((float) (_move_time - 2 * _accel_time));
                    break;
                case DECELERATING:
                    move_progress = (float)(millis() - _move_start_time - _move_time + _accel_time) / ((float) _accel_time);
                    break;
                default:
                    move_progress = 0.0;
                    break;
            }
            
            if (move_progress <= 1.0) {
                // Calculate current speed based on phase
                switch(_move_stage) {
                    case ACCELERATING:
                        _last_speed = move_progress * _target_speed;
                        break;
                    case CRUISING:
                        _last_speed = _target_speed;
                        break;
                    case DECELERATING:
                        _last_speed = (1.0 - move_progress) * _target_speed;
                        break;
                    default:
                        _last_speed = 0.0;
                        break;
                }
                
                // Calculate next position target along the direction vector
                ThreeByOne next_pos = _direction_vector * _last_speed * (static_cast<double>(delta) / 1000.0) + ThreeByOne(_current_cartesian); 
                
                _moving_flag = true;
                
                // Get current axis states
                for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
                    _current_angles[i] = axes[i].getCurrentPos();
                    _current_velocities[i] = axes[i].getCurrentVelocity(); 
                }
                
                // Move to next position via inverse kinematics
                rapidMove(next_pos); //updates _next_angles
                
                // Calculate required velocities and set as feedforward
                for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
                    double next_velocity = (_next_angles[i] - _current_angles[i]) / (static_cast<double>(delta) / 1000.0);
                    double next_acceleration = (next_velocity - _current_velocities[i]) / (static_cast<double>(delta) / 1000.0);
                    axes[i].setFeedforwardVelocity(next_velocity); //rad/s
                }
            }
            else {
                // Transition to next phase or complete move
                switch(_move_stage) {
                    case ACCELERATING:
                        _move_stage = CRUISING;
                        break;
                    case CRUISING:
                        _move_stage = DECELERATING;
                        break;
                    case DECELERATING:
                        // Motion complete
                        for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
                            axes[i].setFeedforwardVelocity(0.0);
                        }
                        
                        _move_stage = STOPPED;
                        break;
                    default:
                        break;
                }
                
                // Final position adjustment after deceleration
                if (_moving_flag && _move_stage == STOPPED) {
                    rapidMove(_end_cartesian[0], _end_cartesian[1], _end_cartesian[2]);
                }
            }
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Initialize a linear move from current position to target with velocity profile
 *
 * Calculates motion timing to reach the target position with smooth acceleration,
 * constant velocity, and deceleration phases. Must call linearMovePerform() repeatedly
 * after this to execute the motion.
 *
 * @param x Target X position [mm]
 * @param y Target Y position [mm]
 * @param z Target Z position [mm]
 * @param target_speed Desired velocity during cruise phase [mm/s]
 * @param relative If true, target is relative to current position (NOT IMPLEMENTED)
 * @return 0 if successful, 1 if requested speed exceeds _max_speed (will be capped)
 *
 * @note After calling this, repeatedly call linearMovePerform() until it returns 0.
 *       The motion uses MAX_LINEAR_ACCELERATION for both accel and decel phases.
 *       If total distance is small, acceleration phase is shortened to fit.
 */
_Bool Leg::linearMoveSetup(double x,  double y, double z, double target_speed, _Bool relative) {
    uint8_t retval = 0;
    
    // Cap speed to maximum allowed
    double speed = target_speed;
    if (target_speed > _max_speed) {
        speed = _max_speed;
        retval = 1; // return warning that speed was capped
    }
    
    // Store starting position
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        _start_cartesian[i] = _current_cartesian[i];
    }
    
    // Set target position
    _end_cartesian[0] = x;
    _end_cartesian[1] = y;
    _end_cartesian[2] = z;
    
    // Calculate total distance to travel
    _move_start_time = millis();
    _moving_flag = true;
    double x_dist = _start_cartesian[0] - _end_cartesian[0];
    double y_dist = _start_cartesian[1] - _end_cartesian[1];
    double z_dist = _start_cartesian[2] - _end_cartesian[2];
    double total_dist = sqrt(x_dist*x_dist + y_dist*y_dist + z_dist*z_dist);
    
    // Calculate acceleration phase timing
    double accel_time_s = speed / MAX_LINEAR_ACCELERATION;
    double accel_distance = 0.5 * MAX_LINEAR_ACCELERATION * accel_time_s * accel_time_s;
    double move_time_s;
    
    // If move is very short, shorten acceleration phase to fit
    if (total_dist < 2.0 * accel_distance) {
        accel_distance = total_dist / 2.0;
        accel_time_s = sqrt(2.0 * accel_distance / MAX_LINEAR_ACCELERATION);
        move_time_s = 2 * accel_time_s;  // Total time = accel + decel only
        _target_speed = accel_time_s * MAX_LINEAR_ACCELERATION; // Peak speed achieved
    }
    else {
        // Standard trapezoidal profile
        move_time_s = 
            ((total_dist - (2 * accel_distance)) / speed   // Constant speed phase time
            + (2 * speed) / MAX_LINEAR_ACCELERATION);       // Accel + decel time
        _target_speed = speed;
    }
    
    _last_speed = 0.0;
    _move_stage = move_stage::ACCELERATING;
    _accel_time = static_cast<uint32_t>(accel_time_s * 1000.0);
    _move_time = static_cast<uint32_t>(move_time_s * 1000.0);

    // Calculate direction vector (normalized)
    _direction_vector = ThreeByOne(_end_cartesian[0] - _start_cartesian[0], 
                                   _end_cartesian[1] - _start_cartesian[1], 
                                   _end_cartesian[2] - _start_cartesian[2]).unit_vector();
     
    return retval;
}

/**
 * @brief Apply target angles to all axes
 *
 * Copies angles from _next_angles[] (calculated by inverse kinematics)
 * to both the axis setpoints and the public current_angles[] array.
 */
void Leg::_moveAxes() {
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        axes[i].setTargetPos(_next_angles[i]);
        current_angles[i] = _next_angles[i];
    }
}

void Leg::setAxisTargetPos(uint8_t axis_number, double pos) {
    axes[axis_number].setTargetPos(pos);
}

void Leg::stopAxis(uint8_t axis_number) {
    axes[axis_number].allowMotion(false);
}