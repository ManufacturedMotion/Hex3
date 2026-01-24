#include "leg.hpp"
#include "hexapod.hpp"
#include "axis.hpp"
#include "config.hpp"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "three_by_matrices.hpp"
#include <Arduino.h>
#include "log_levels.hpp"
#include "mux.hpp"


double zero_points[NUM_LEGS][NUM_AXES_PER_LEG] = ZERO_POINTS;
double min_pos[NUM_LEGS][NUM_AXES_PER_LEG] = MIN_POS;
double max_pos[NUM_LEGS][NUM_AXES_PER_LEG] = MAX_POS;
double scale_fact[NUM_LEGS][NUM_AXES_PER_LEG] = SCALE_FACT;
_Bool reverse_axis[NUM_LEGS][NUM_AXES_PER_LEG] = REVERSE_AXIS;

enum Dimension { X = 0, Y = 1, Z = 2};

Leg::Leg() {
    _leg_number = 0;
}

void Leg::begin(){
    mux.begin();
    axes[0].link(D11, D12, 5, mux);
    axes[1].link(D18, D2, D17, D3, 6, mux);
    axes[2].link(D16, D15, 7, mux);
    pinMode(TOE_PIN, INPUT); 
}

void Leg::initializeAxes(uint8_t leg_number) {
    _leg_number = leg_number;
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        axes[i].initializePositionLimits(min_pos[_leg_number][i], max_pos[_leg_number][i]);
        axes[i].setMapping(zero_points[_leg_number][i], scale_fact[_leg_number][i], reverse_axis[_leg_number][i]);
    }
}

void Leg::runSpeed() {
    // for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
    //     axes[j].runSpeed();
    // }
    static uint32_t last_print_time = 0;
    if (millis() - last_print_time > 1000) {
        last_print_time = millis();
        Serial.printf("Leg %d positions: Axis0: %f; Axis1: %f; Axis2: %f\n", _leg_number, axes[0].getCurrentPos(), axes[1].getCurrentPos(), axes[2].getCurrentPos());
        Serial.printf("Leg %d velocities: Axis0: %f; Axis1: %f; Axis2: %f\n", _leg_number, axes[0].getCurrentVelocity(), axes[1].getCurrentVelocity(), axes[2].getCurrentVelocity());
        Serial.printf("Leg %d accelerations: Axis0: %f; Axis1: %f; Axis2: %f\n", _leg_number, axes[0].getCurrentAcceleration(), axes[1].getCurrentAcceleration(), axes[2].getCurrentAcceleration());
    }
    for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
        axes[j].trackMotion();
        axes[j].moveToPos();
    }
}

void Leg::stopAxis(uint8_t axis_number) {
    axes[axis_number].allowMotion(false);
}

void Leg::setAxisTargetPos(uint8_t axis_number, double pos) {
    axes[axis_number].allowMotion(true);
    axes[axis_number].setTargetPos(pos);
}

void Leg::setAxisControlConstants(uint8_t axis_number, double Kp, double Ki, double Kd, double Kv_ff, double Ka_ff) {
    axes[axis_number].setControlConstants(Kp, Ki, Kd, Kv_ff, Ka_ff);
}

void Leg::setAxisDutyCycle(uint8_t axis_number, bool dir, float duty_cycle) {
    axes[axis_number].setDutyCycle(dir, duty_cycle);
}

_Bool Leg::toePressed(){
    _Bool is_pressed = false;
    if (analogRead(TOE_PIN) < toe_threshold)
        is_pressed = true;
    return is_pressed;
}

_Bool Leg::_inverseKinematics(double x, double y, double z) {

    if (!_checkSafeCoords(x,y,z))
        return false; 
    double potential_results[3];
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
        potential_results[0] = atan2(x,y);
    }

    double y_virtual_planar = sqrt(y * y + x * x) - _length0;
    double planar_distance = sqrt(y_virtual_planar * y_virtual_planar + z * z);
    
    double theta2_tool  =   (planar_distance*planar_distance - _length1*_length1 - _length2*_length2)
                        /   (2 * _length1 * _length2);

    potential_results[2] = atan2(sqrt(1 - (theta2_tool*theta2_tool)), theta2_tool);
    
    // if (z < 0) {
    //     potential_results[2] = -potential_results[2];
    //     double theta1_tool1 = atan2(_length2 * sin(potential_results[2]), _length1 + _length2 * cos(potential_results[2]));
    //     double theta1_tool0 = atan2(z, y_virtual_planar);
    //     potential_results[1] = theta1_tool0 + theta1_tool1;
    // }
    // else {
        double theta1_tool0 = atan2(z, y_virtual_planar);
        double theta1_tool1 = atan2(_length2 * sin(potential_results[2]), _length1 + _length2 * cos(potential_results[2]));
        potential_results[1] = theta1_tool0 - theta1_tool1;
    // }
    if (z < 0) {
        potential_results[2] = -potential_results[2];
        potential_results[1] = theta1_tool0 + theta1_tool1;
    }
    
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        if (potential_results[i] != potential_results[i]) { //Check for NaN
            // Serial.println(i);
            // Serial.println(potential_results[i]);
            return false;
        }   
        #ifdef LEG_DEBUG

            
            if (potential_results[i] < min_pos[_leg_number][i] || potential_results[i] > max_pos[_leg_number][i]) {
                #ifdef LEG_DEBUG
                    Serial.printf("Potential result %d: %f is ", i, potential_results[i]);
                    Serial.println("OUT OF BOUNDS");
                #endif
            }
            else {
                _next_angles[i] = potential_results[i];
            }
        #endif
    }
    
    // ThreeByOne resulting_pos = forwardKinematics(potential_results[0], potential_results[1], potential_results[2]);
    // Serial.printf("Result\n  x: %f; y: %f; z: %f\n", resulting_pos.values[0], resulting_pos.values[1], resulting_pos.values[2]);

    // Serial.printf("Result\n  angle0: %f; angle1: %f; angle2: %f\n", potential_results[0], potential_results[1], potential_results[2]);
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        
    }
    return true;
}

_Bool Leg::_checkSafeCoords(double x, double y, double z) {
    //If coords within robot body
    //  return false;
    
    return true;
}

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

uint8_t Leg::linearMovePerform() {
    uint32_t delta = millis() - _last_linear_move_time;
    if (delta >= LINEAR_MOVE_INTERVAL_MS) {
        _last_linear_move_time = millis();
        double move_progress = (float)(millis() - _move_start_time) / ((float) _move_time);
        if (move_progress <= 1.0) {
            // Serial.printf("Move start time: %d, _move_time %d\n", millis() - _move_start_time, _move_time);
            double next_x = move_progress * (_end_cartesian[0] - _start_cartesian[0]) + _start_cartesian[0];
            double next_y = move_progress * (_end_cartesian[1] - _start_cartesian[1]) + _start_cartesian[1];
            double next_z = move_progress * (_end_cartesian[2] - _start_cartesian[2]) + _start_cartesian[2];
            _moving_flag = true;
            for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
                _current_angles[i] = axes[i].getCurrentPos();
                _current_velocities[i] = axes[i].getCurrentVelocity(); 
            }
            rapidMove(next_x, next_y, next_z); //updates _next_angles
            for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
                axes[i].setTargetVelocity((_next_angles[i] - _current_angles[i]) / (static_cast<double>(delta) * 1000.0)); //rad/s
                // axes[i].setTargetAcceleration((_current_velocities[i] - ))
            }
        }
        else {
            for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
                axes[i].setTargetVelocity(0.0); //rad/s
                // axes[i].setTargetAcceleration((_current_velocities[i] - ))
            }
            _moving_flag = false;
        }
        return 1;
    }
    return 0;
}

void Leg::wait(uint32_t time_ms) {
   
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        _start_cartesian[i] = _current_cartesian[i];
        _end_cartesian[i] = _current_cartesian[i];
    }
    _move_start_time = millis();
    _moving_flag = true;
    _move_time = time_ms;
    // Serial.printf("Wait for %d ms\n", _move_time);
}


// double Leg::linearMovement(double move_progress, Dimension axis) {
//     return move_progress * (_end_cartesian[axis] - _start_cartesian[axis]) + _start_cartesian[axis];
// }

// double Leg::radialMovement(double move_progress, Dimension axis) {
//     return move_progress * 
// }

// void Leg::ISRLinearMove() {
//     // Attach this function to an IntervalTimer on a 10-50ms interval
//     uint8_t move_status = runLegSpeed(&linearMovement);
//     if (!move_status)
//         _moving_flag = false;
// }

_Bool Leg::isMoving() {
    return _moving_flag;
}

_Bool Leg::linearMoveSetup(double x,  double y, double z, double target_speed, _Bool relative) {
    uint8_t retval = 0;
    double speed = target_speed;
    if (target_speed > _max_speed) {
        speed = _max_speed;
        retval = 1; // move speed capped
    }
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        _start_cartesian[i] = _current_cartesian[i];
    }
    _end_cartesian[0] = x;
    _end_cartesian[1] = y;
    _end_cartesian[2] = z;
    // if (relative) {
    //     _end_cartesian[0] += _current_cartesian[0];
    //     _end_cartesian[1] += _current_cartesian[1];
    //     _end_cartesian[2] += _current_cartesian[2];
    // }
    _move_start_time = millis();
    _moving_flag = true;
    double x_dist = _start_cartesian[0] - _end_cartesian[0];
    double y_dist = _start_cartesian[1] - _end_cartesian[1];
    double z_dist = _start_cartesian[2] - _end_cartesian[2];
    _move_time = (sqrt(x_dist*x_dist + y_dist*y_dist + z_dist*z_dist) / speed) * 1000; 
    // Serial.printf("_end_cartesian: x:%f, y:%f, z:%f\n", _end_cartesian[0], _end_cartesian[1], _end_cartesian[2]);
    return retval;
}

void Leg::_moveAxes() {
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        axes[i].setTargetPos(_next_angles[i]);
        current_angles[i] = _next_angles[i];
    }
}

ThreeByOne Leg::forwardKinematics(double axis0_angle, double axis1_angle, double axis2_angle) {
    ThreeByOne length0(0.0, _length0, 0.0);
    ThreeByOne length1(0.0, _length1, 0.0);
    ThreeByOne length2(0.0, _length2, 0.0);

    length0.rotateYaw(axis0_angle);

    length1.rotateYaw(axis0_angle);
    length1.rotateRoll(axis1_angle);

    length2.rotateYaw(axis0_angle);
    length2.rotateRoll(axis1_angle);
    length2.rotateRoll(axis2_angle);

    // Serial.printf("length0\n  x: %f; y: %f; z: %f\n",length0.values[0], length0.values[1], length0.values[2]);
    // Serial.printf("length1\n  x: %f; y: %f; z: %f\n",length1.values[0], length1.values[1], length1.values[2]);
    // Serial.printf("length2\n  x: %f; y: %f; z: %f\n",length2.values[0], length2.values[1], length2.values[2]);
    return length0 + length1 + length2;
}

ThreeByOne Leg::getCurrentPosition() {
    return ThreeByOne(_current_cartesian);
}

ThreeByOne Leg::getEndPosition() {
    return ThreeByOne(_end_cartesian);
}

