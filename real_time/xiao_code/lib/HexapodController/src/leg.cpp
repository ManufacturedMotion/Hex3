#include "leg.hpp"
#include "hexapod.hpp"
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


double zero_points[NUM_LEGS][NUM_AXES_PER_LEG] = ZERO_POINTS;
double min_pos[NUM_LEGS][NUM_AXES_PER_LEG] = MIN_POS;
double max_pos[NUM_LEGS][NUM_AXES_PER_LEG] = MAX_POS;
double scale_fact[NUM_LEGS][NUM_AXES_PER_LEG] = SCALE_FACT;
_Bool reverse_axis[NUM_LEGS][NUM_AXES_PER_LEG] = REVERSE_AXIS;

enum Dimension { X = 0, Y = 1, Z = 2};

Leg::Leg() {
    _leg_number = 0;
    can = nullptr;
}
// 2B, 2A, 4B, 6A, 5A, 2B, 6A, 7B
void Leg::begin(){
    mux.begin();
    axes[0].link(D11, D12, 5, mux);
    axes[1].link(D18, D2, D17, D3, 6, mux);
    axes[2].link(D16, D15, 7, mux);
    pinMode(TOE_PIN, INPUT); 
    if (can){
        can->begin();
    }
}

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

void Leg::_trackMotion() {
    for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
        axes[j].trackMotion();
    }
    if (millis() - _last_pos_update_time > LEG_POSITION_TRACK_INTERVAL_MS) {
        _last_pos_update_time = millis();
        _forwardKinematics(axes[0].getCurrentPos(), axes[1].getCurrentPos(), axes[2].getCurrentPos(), _current_pos[X], _current_pos[Y], _current_pos[Z]);
    }
        if (millis() - _last_velocity_update_time > LEG_VELOCITY_TRACK_INTERVAL_MS) {
            uint32_t delta_time = millis() - _last_velocity_update_time;
            _last_velocity_update_time = millis();
            for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
                _last_velocity[j] = _current_velocity[j];
                double distance_traversed = _current_pos[j] - _last_pos[j];
                _current_velocity[j] = distance_traversed / (static_cast<double>(delta_time) / 1000.0); //units/s
                _current_acceleration[j] = (_current_velocity[j] - _last_velocity[j]) / (static_cast<double>(delta_time) / 1000.0); //units/s^
                _last_pos[j] = _current_pos[j];
            }
        }
}

void Leg::runSpeed() {
    // for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
    //     axes[j].runSpeed();
    // }
    static uint32_t last_print_time = 0;
    if (millis() - last_print_time > 10) {
        last_print_time = millis();
        // Serial.printf("{\"Axis0\": {\"pos\": %f, \"vel\": %f, \"acc\": %f, \"duty\": %f, \"torque\": %f}, \"Axis1\": {\"pos\": %f, \"vel\": %f, \"acc\": %f, \"duty\": %f, \"torque\": %f}, \"Axis2\": {\"pos\": %f, \"vel\": %f, \"acc\": %f, \"duty\": %f, \"torque\": %f}, \"voltage\": %f}\n", 
        //     _current_pos[X], _current_velocity[X], _current_acceleration[X], axes[0].getDutyCycle(), axes[0].getEstimatedTorque(),
        //     _current_pos[Y], _current_velocity[Y], _current_acceleration[Y], axes[1].getDutyCycle(), axes[1].getEstimatedTorque(),
        //     _current_pos[Z], _current_velocity[Z], _current_acceleration[Z], axes[2].getDutyCycle(), axes[2].getEstimatedTorque(),
        //     voltage_sensor.filteredRead());    
        Serial.printf("{\"Axis0\": {\"pos\": %f, \"vel\": %f, \"acc\": %f, \"duty\": %f, \"torque\": %f}, \"Axis1\": {\"pos\": %f, \"vel\": %f, \"acc\": %f, \"duty\": %f, \"torque\": %f}, \"Axis2\": {\"pos\": %f, \"vel\": %f, \"acc\": %f, \"duty\": %f, \"torque\": %f}, \"voltage\": %f}\n", 
        axes[0].getCurrentPos(), axes[0].getCurrentVelocity(), axes[0].getCurrentAcceleration(), axes[0].getDutyCycle(), axes[1].getEstimatedTorque(),
            axes[1].getCurrentPos(), axes[1].getCurrentVelocity(), axes[1].getCurrentAcceleration(), axes[1].getDutyCycle(), axes[1].getMOBDisturbanceTorque(),
            axes[2].getCurrentPos(), axes[2].getCurrentVelocity(), axes[2].getCurrentAcceleration(), axes[2].getDutyCycle(), axes[1].getEstimatedTorque(), 
            voltage_sensor.filteredRead());    
    }
    for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
        axes[j].moveToPos();
        // axes[j].moveAtVelocity();
        axes[j].setInputVoltage(voltage_sensor.filteredRead());
    }
    _trackMotion();
}

void Leg::stopAxis(uint8_t axis_number) {
    axes[axis_number].allowMotion(false);
}

void Leg::setAxisTargetPos(uint8_t axis_number, double pos) {
    axes[axis_number].allowMotion(true);
    axes[axis_number].setTargetPos(pos);
}

void Leg::setAxisControlConstants(uint8_t axis_number, double tuning_voltage, double Kp_pos, double Kd_pos, double Kp_vel, double Ki_vel, double Kv_ff) {
    axes[axis_number].setControlConstants(tuning_voltage, Kp_pos, Kd_pos, Kp_vel, Ki_vel, Kv_ff);
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

_Bool Leg::_forwardKinematics(double theta0, double theta1, double theta2, double& x, double& y, double& z) {
    double planar_distance = _length1 * cos(theta1) + _length2 * cos(theta1 + theta2) + _length0;
    x = planar_distance * sin(theta0);
    y = planar_distance * cos(theta0);
    z = -_length1 * sin(theta1) - _length2 * sin(theta1 + theta2);
    return true;
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

_Bool Leg::rapidMove(ThreeByOne target_pos) {
    return rapidMove(target_pos.values[0], target_pos.values[1], target_pos.values[2]);
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
        if (_move_stage == move_stage::ACCELERATING || _move_stage == move_stage::CRUISING || _move_stage == move_stage::DECELERATING) {
            double move_progress;
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
                // Serial.printf("Move start time: %d, _move_time %d\n", millis() - _move_start_time, _move_time);
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
                ThreeByOne next_pos = _direction_vector * _last_speed * (static_cast<double>(delta) / 1000.0) + ThreeByOne(_current_cartesian); 
                // Serial.printf("Moving to %f, %f, %f; delta: %d; current_speed: %f\n", next_pos.values[0], next_pos.values[1], next_pos.values[2], delta, _last_speed);
                _moving_flag = true;
                for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
                    _current_angles[i] = axes[i].getCurrentPos();
                    _current_velocities[i] = axes[i].getCurrentVelocity(); 
                }
                rapidMove(next_pos); //updates _next_angles
                for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
                    double next_velocity = (_next_angles[i] - _current_angles[i]) / (static_cast<double>(delta) / 1000.0);
                    double next_acceleration = (next_velocity - _current_velocities[i]) / (static_cast<double>(delta) / 1000.0);
                    axes[i].setFeedforwardVelocity(next_velocity); //rad/s
                }
            }
            else {
                
                switch(_move_stage) {
                    case ACCELERATING:
                        _move_stage = CRUISING;
                        break;
                    case CRUISING:
                        _move_stage = DECELERATING;
                        break;
                    case DECELERATING:
                        for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
                            axes[i].setFeedforwardVelocity(0.0);
                        }
                        
                        _move_stage = STOPPED;
                        break;
                    default:
                        break;
                }
                if (_moving_flag && _move_stage == STOPPED) {
                    rapidMove(_end_cartesian[0], _end_cartesian[1], _end_cartesian[2]);
                }
            }
            return 1;
        }
    }
    return 0;
}

void Leg::setAxisTargetVelocity(uint8_t axis_number, double target_velocity) {
    axes[axis_number].setTargetVelocity(target_velocity);
    Serial.println("Set target velocity for axis " + String(axis_number) + ": " + String(target_velocity));
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
    double total_dist = sqrt(x_dist*x_dist + y_dist*y_dist + z_dist*z_dist);
    double accel_time_s = speed / MAX_LINEAR_ACCELERATION;
    double accel_distance = 0.5 * MAX_LINEAR_ACCELERATION * accel_time_s * accel_time_s;
    double move_time_s;
    if (total_dist < 2.0 * accel_distance) {
        accel_distance = total_dist / 2.0;
        accel_time_s = sqrt(2.0 * accel_distance / MAX_LINEAR_ACCELERATION);
        move_time_s = 2 * accel_time_s;
        _target_speed = accel_time_s * MAX_LINEAR_ACCELERATION; //mm/s
    }
    else {
        move_time_s = 
            ((total_dist - (2 * accel_distance)) / speed // Constant speed time
            + (2 * speed) / MAX_LINEAR_ACCELERATION); // Accel/decel time
        _target_speed = speed;
    }
    _last_speed = 0.0;
    _move_stage = move_stage::ACCELERATING;
    _accel_time = static_cast<uint32_t>(accel_time_s * 1000.0);
    _move_time = static_cast<uint32_t>(move_time_s * 1000.0);

    _direction_vector = ThreeByOne(_end_cartesian[0] - _start_cartesian[0], _end_cartesian[1] - _start_cartesian[1], _end_cartesian[2] - _start_cartesian[2]).unit_vector();
    Serial.printf("Linear move setup: distance: %f, target speed: %f, accel time: %d ms, total move time: %d ms; accel distance: %f\n", total_dist, _target_speed, _accel_time, _move_time, accel_distance);
     
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

