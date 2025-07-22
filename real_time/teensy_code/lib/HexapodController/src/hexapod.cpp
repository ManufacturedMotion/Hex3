#include "leg.hpp"
#include "axis.hpp"
#include "hexapod.hpp"
#include "position.hpp"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "three_by_matrices.hpp"
#include <Arduino.h>
#include "log_levels.hpp"

Hexapod::Hexapod() 
{ 
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        legs[i].initializeAxes(i);
		_leg_queues[i].setLeg(&legs[i]);
    }
}

void Hexapod::startUp() {

}

void Hexapod::moveLegAxisToPos(uint8_t leg_number, uint8_t axis_number, double target_position) {
	legs[leg_number].axes[axis_number].moveToPos(target_position);
}

_Bool Hexapod::moveLegToPos(uint8_t leg_number, double x, double y, double z) {
	return legs[leg_number].rapidMove(x, y, z);
}

void Hexapod::moveToZeros() {
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
			legs[i].axes[j].moveToPos(0);
		}        
    }
} 

void Hexapod::detachAllServos() {
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
			legs[i].axes[j].detachServo();
		}        
    }
}

void Hexapod::sit() {
	Position sit_pos;
	sit_pos.set(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);
	enqueueLinearMove(sit_pos, 100.0);
}

void Hexapod::dance0() {

	uint32_t dance_time = 0;
	Position buffer0;
	buffer0.setPos(_step_queue.getCurrentQueueEndPos());
	buffer0.x = 0.00;
	buffer0.y = 0.00;
	buffer0.yaw = 0.00;
	dance_time += _step_queue.enqueue(buffer0, 100, StepType::RETURN_TO_NEUTRAL);

	buffer0.setPos(_step_queue.getCurrentQueueEndPos());
	buffer0.z = 150.00;
	dance_time += enqueueLinearMove(buffer0, 150.0);
	
	buffer0.set(0.00, 0.00, 150.00, 50.0, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, 150.0);
	
	buffer0.set(0.00, 0.00, 150.00, 0.00, 50.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, 150.0);
	
	buffer0.set(0.00, 0.00, 150.00, -50.00, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, 150.0);

	buffer0.set(0.00, 0.00, 150.00, 0.00, -50.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, 150.0);

	buffer0.set(0.00, 0.00, 150.00, 50.00, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, 150.0);

	buffer0.set(0.00, 0.00, 150.00, 0.00, -50.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, 150.0);

	buffer0.set(0.00, 0.00, 150.00, -50.00, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, 150.0);

	buffer0.set(0.00, 0.00, 150.00, 0.00, 50.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, 150.0);

	buffer0.set(0.00, 0.00, 150.00, 50.0, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, 150.0);

	buffer0.set(0.00, 0.00, 150.00, 0.00, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, 150.0);

}

void Hexapod::dance1() {

	double dance_speed = 150;
	uint32_t dance_time = 0;
	Position buffer0;
	buffer0.setPos(_step_queue.getCurrentQueueEndPos());
	buffer0.x = 0.00;
	buffer0.y = 0.00;
	buffer0.yaw = 0.00;
	dance_time += _step_queue.enqueue(buffer0, dance_speed, StepType::RETURN_TO_NEUTRAL);
	
	buffer0.set(0.00, 0.00, 200.00, 0.00, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, dance_speed);
	
	buffer0.set(0.00, 0.00, 100.00, 0.00, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, dance_speed);
	
	buffer0.set(0.00, 75.0, 150.00, 0.00, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, dance_speed);
	
	buffer0.set(75.00, 0.0, 150.00, 0.00, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, dance_speed);

	buffer0.set(0.00, -75.00, 150.00, 0.00, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, dance_speed);

	buffer0.set(-75.00, 00.00, 150.00, 0.00, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, dance_speed);

	buffer0.set(0.00, 0.00, 150.00, 0.00, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, dance_speed);

	buffer0.set(0.00, 0.00, 150.00, 0.00, 0.00, 0.00);
	dance_time += enqueueLinearMove(buffer0, dance_speed);
}

void Hexapod::dance2() {
    
    //NOT IMPLEMENTED YET
    return;

}

void Hexapod::dance3() {
    
    //NOT IMPLEMENTED YET
    return;

}

void Hexapod::dance4() {
    
    //NOT IMPLEMENTED YET
    return;

}

void Hexapod::dance5() {
    
    //NOT IMPLEMENTED YET
    return;

}
            
void Hexapod::stand() {
	Position stand_pos;
	stand_pos.set(0.0, 0.0, 150.0, 0.0, 0.0, 0.0);
	enqueueRapidMove(stand_pos);
}

void Hexapod::rapidMove(double x, double y, double z, double roll, double pitch, double yaw) {
	Position pos;
	pos.set(x, y, z, roll, pitch, yaw);
	rapidMove(pos);
	_current_pos.setPos(pos);
}

void Hexapod::rapidMove(Position next_pos) {
	_Bool active_legs[NUM_LEGS];
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		active_legs[i] = true;
	}
	rapidMove(next_pos, active_legs);
}

void Hexapod::rapidMove(Position next_pos, _Bool active_legs[NUM_LEGS], _Bool update_current_pos) {
	// Updated to use new _inverseKinematics caller, but not tested since this
	// function is not currently used
	ThreeByOne next_positions[NUM_LEGS];
	_inverseKinematics(next_pos, next_positions);
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
			if (active_legs[i]) {
				_next_leg_pos[i][j] = next_positions[i].values[j];
			}
			#ifdef SERIAL_CSV_DEBUG
				Serial.print(_next_leg_pos[i][j]);
				Serial.print(",");
			#endif
		}
	}
	#ifdef SERIAL_CSV_DEBUG
		Serial.print('\n');
	#endif
	_moveLegs();
	if (update_current_pos) {
		_current_pos.setPos(next_pos);
	}
}

uint8_t Hexapod::linearMoveSetup(double x, double y, double z, double roll, double pitch, double yaw, double target_speed, _Bool active_legs[NUM_LEGS]) {
	_end_pos.set(x, y, z, roll, pitch, yaw);
	_start_pos.setPos(_current_pos);
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		_linear_move_legs[i] = active_legs[i];
	}
	uint8_t retval = 0;
    double speed = target_speed;
    if (target_speed > _max_speed) {
        speed = _max_speed;
        retval = 1; // move speed capped
    }
	_move_progress = 0;
	_move_start_time = millis();
	Position pos_delta = _end_pos - _start_pos;
	_move_time = (fabs(pos_delta.magnitude()) / speed) * 1000; //convert to seconds
	_moving_flag = true;
	return retval;
}

uint8_t Hexapod::linearMoveSetup(Position next_pos, double target_speed, _Bool active_legs[NUM_LEGS]) {
	_end_pos.setPos(next_pos); 
	_start_pos.setPos(_current_pos);
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		_linear_move_legs[i] = active_legs[i];
	}
	uint8_t retval = 0;
	double speed = target_speed;
	if (target_speed > _max_speed) {
		speed = _max_speed;
		retval = 1; // move speed capped
	}
	_move_progress = 0;
	_move_start_time = millis();
	Position pos_delta = _end_pos - _start_pos;
	_move_time = (fabs(pos_delta.magnitude()) / speed) * 1000; //convert to milliseconds
	_moving_flag = true;
	return retval;
}

uint8_t Hexapod::linearMoveSetup(Position next_pos, double target_speed) {
	_Bool active_legs[NUM_LEGS];
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		active_legs[i] = true;
	}
	return linearMoveSetup(next_pos, target_speed, active_legs);
}

uint8_t Hexapod::linearMoveSetup(double x, double y, double z, double roll, double pitch, double yaw, double target_speed) {
	_Bool active_legs[NUM_LEGS];
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		active_legs[i] = true;
	}
	return linearMoveSetup(x, y, z, roll, pitch, yaw, target_speed, active_legs);
}

uint8_t Hexapod::legLinearMoveSetup(uint8_t leg, double x,  double y, double z, double target_speed, _Bool relative) {
	ThreeByOne coord = ThreeByOne(x, y, z);
	coord.rotateYaw(_home_yaws[leg]);
	if (!relative) {
		coord += _stance_offset;
	}
	if (relative) {
		coord += ThreeByOne(_next_leg_pos[leg]);
	}
	_next_leg_pos[leg][0] = coord.values[0];
	_next_leg_pos[leg][1] = coord.values[1];
	_next_leg_pos[leg][2] = coord.values[2];
	
	return legs[leg].linearMoveSetup(coord.values[0], coord.values[1], coord.values[2], target_speed, relative);
}

uint8_t Hexapod::legLinearMoveSetup(uint8_t leg, ThreeByOne end_coord, double target_speed, _Bool relative) {
	return legLinearMoveSetup(leg, end_coord.values[0], end_coord.values[1], end_coord.values[2], target_speed, relative);
}

void Hexapod::opQueueTest() {
	ThreeByOne coord1 = ThreeByOne(0.0,   0.0,   100.0);
	ThreeByOne coord5 = ThreeByOne(0.0,   0.0,   100.0);
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		_leg_queues[i].enqueue(coord1, 100.0, false);
		_leg_queues[i].enqueue(ThreeByOne(0.0, 0.0, 0.0), 100.0, true, 1000);
		_leg_queues[i].enqueue(coord5, 100.0, true);
	}
}

void Hexapod::linearMovePerform() {
	if (isLowLevelBusy()) {
		double move_progress = (float)(millis() - _move_start_time) / ( _move_time);
		if (move_progress <= 1.0) {
			Position next_pos = (_end_pos - _start_pos) * move_progress + _start_pos;
			rapidMove(next_pos, _linear_move_legs);
			#if LOG_LEVEL >= CALCULATION_LOGGING
				Serial.println("linear move legs: " + String(_linear_move_legs[0]) + " " + String(_linear_move_legs[1]) + " " + String(_linear_move_legs[2]) + " "
				+ String(_linear_move_legs[3]) + " " + String(_linear_move_legs[4]) + " " + String(_linear_move_legs[5]) + "\n");
			#endif
		}
		for (uint8_t i = 0; i < NUM_LEGS; i++) {
			if (legs[i].isMoving()) {
				legs[i].linearMovePerform();
			}
		}
		if (move_progress > 1.0) {
			_moving_flag = false;
		}
	}
}

void Hexapod::_moveLegs() {
	#if LOG_LEVEL >= CALCULATION_LOGGING
		Serial.println("Next leg positions]");
	#endif
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if (!legs[i].isMoving()) {
        	legs[i].rapidMove(_next_leg_pos[i][0], _next_leg_pos[i][1],_next_leg_pos[i][2]);
			#if LOG_LEVEL >= CALCULATION_LOGGING
				Serial.println("leg: " + String(i) + " x:" + String(_next_leg_pos[i][0]) + " y:" + String(_next_leg_pos[i][1]) + " z:" + String(_next_leg_pos[i][2]) + "\n");
			#endif
		}
	}
}

double Hexapod::get_max_step_magnitude() {
	Position current_pos = _step_queue.getCurrentQueueEndPos();
	return MAX_STEP_MAGNITUDE - sqrt(pow((current_pos.z - 100.0) / 2.0, 2) + pow(current_pos.roll, 2) + pow(current_pos.pitch, 2)) / 2.0;
}

uint8_t Hexapod::walkSetup(double x, double y, double speed, _Bool return_to_neutral) {
	ThreeByOne relative_end_coord = ThreeByOne(x, y, 0.0);
	return walkSetup(relative_end_coord, speed, return_to_neutral);
}

uint8_t Hexapod::enqueueRapidMove(Position pos) {
	return _step_queue.enqueue(pos, 0.0, StepType::RAPID_MOVE);
}

uint32_t Hexapod::enqueueLinearMove(Position next_pos, double target_speed, bool relative) {
	// Enqueue a linear move to the step queue
	// This will be performed by the low-level controller
	return _step_queue.enqueue(next_pos, target_speed, relative ? StepType::LINEAR_MOVE_RELATIVE : StepType::LINEAR_MOVE_ABSOLUTE);
}

double Hexapod::_getMaxStepMagnitudeInDirection(Position direction_vector, _Bool flipped_step_group) {	
	Position buffer1;
	Position buffer2;
	// Create a vector of the next step that ends up with the max step magnitude in the direction of the move
	// and enqueue it

	// Constraints:
	// 1. The step must be in the direction of the move
	// 2. The step should end up in a position that is equal to the max step magnitude
	// | _step_queue.getCurrentQueueEndPos() + step | = MAX_STEP_MAGNITUDE
	// 3. The step should be in the direction of the move
	
	buffer1.setPos(_step_queue.getCurrentQueueEndPos());
	buffer1.z = 0.00; // For now we don't consider Z, roll, or pitch
	buffer1.roll = 0.00;
	buffer1.pitch = 0.00;
	if (flipped_step_group) {
		buffer1 *= -1.0; // If the step group has been flipped, then the previous step was in the opposite direction
	}
	
	buffer2 = direction_vector.unitVector();
	buffer2.z = 0.00; // For now we don't consider Z, roll, or pitch
	buffer2.roll = 0.00;
	buffer2.pitch = 0.00;
	// buffer2.yaw *= ROTATION_MAGNITUDE_SCALE; // Scale yaw to have a similar range as x and y

	double c = pow(buffer1.x, 2) + pow(buffer1.y, 2) + pow(buffer1.yaw, 2) - pow(get_max_step_magnitude(), 2);
	double b = 2.0 * (buffer1.x * buffer2.x + buffer1.y * buffer2.y + buffer1.yaw * buffer2.yaw);
	double a = pow(buffer2.x, 2) + pow(buffer2.y, 2) + pow(buffer2.yaw, 2);

	// Solve the quadratic equation for the step magnitude
	double discriminant = pow(b, 2) - 4.0 * a * c;
	double step_magnitude;
	if (discriminant < 0.0) {
		// No solution, return 0
		step_magnitude = 0.00;
	}
	else if (fabs(discriminant) <= 0.001) {
		// One solution, use it
		step_magnitude = -b / (2.0 * a);
	}
	else {
		// Two solutions, use the positive one (if there is one)
		step_magnitude = ((-b + sqrt(discriminant)) / (2.0 * a)) > 0.00 ? (-b + sqrt(discriminant)) / (2.0 * a) : (-b - sqrt(discriminant)) / (2.0 * a);
	}
	return step_magnitude;
}

void Hexapod::setWalkVelocity(Position velocity) {
	_walk_velocity.setPos(velocity);
}

uint32_t Hexapod::enqueueMaxStepInDirection(Position direction_vector, double scalar) {
	direction_vector.z = 0.00; // For now we don't consider Z, roll, or pitch
	direction_vector.roll = 0.00;
	direction_vector.pitch = 0.00;
	if (direction_vector.magnitude() < 0.001) {
		return 0; // No step to take
	}

	double speed = _walk_velocity.magnitude();
	uint32_t walk_time = 0;
	double max_step_magnitude_with_flip = _getMaxStepMagnitudeInDirection(direction_vector, true);
	double max_step_magnitude_without_flip = _getMaxStepMagnitudeInDirection(direction_vector, false);
	double max_step_magnitude = max_step_magnitude_without_flip;
	if (max_step_magnitude_with_flip > max_step_magnitude_without_flip) {
		_next_step_type = static_cast<decltype(_next_step_type)>(static_cast<uint8_t>(_next_step_type) ^ 1);
		max_step_magnitude = max_step_magnitude_with_flip;
	}

	if (max_step_magnitude < get_max_step_magnitude()) {
		Position buffer0;
		buffer0.setPos(_step_queue.getCurrentQueueEndPos());
		buffer0.x = 0.00;
		buffer0.y = 0.00;
		buffer0.yaw = 0.00;
		walk_time += _step_queue.enqueue(buffer0, speed, StepType::RETURN_TO_NEUTRAL);
	}
	
	Position step_vector = direction_vector.unitVector() * max_step_magnitude;
	walk_time += _step_queue.enqueue(step_vector * fabs(scalar), speed, _next_step_type);
	return walk_time;
}

uint32_t Hexapod::walkSetup(Position relative_end_pos, double speed) {
	/*
		For every move we must do:
			1. [x] Check if move magnitude is 0
			  	- If so, return 0
			2. [x] Determine if destination is reachable
			  	- X, Y, and Yaw have no limits
				- Z, Roll, and Pitch DO
				- If not reachable, return 0
			3. [x] Determine if move must be broken up into one or more steps
				- If not, enqueue linearMove and return time it takes
			4. [x] Determine if return to neutral necessary before taking steps in commanded direction
			    - If yes, enqueue a return to neutral position first
			5. [x] Break up move into N steps
			  	- After completing end steps the COM of the hexapod should have moved the distance commanded 
				- Keep track of total move time to be returned (including return to neutral if necessary)
		For every step we must do:
			6. [x] Determine if step group should be changed
			7. [x] Determine length of next step (either distance to end point or distance to MAX_STEP_MAGNITUDE in direction of motion)
				- [ ] May be able to reuse length of previous step if the step magnitude has remained constant for two steps in a row and is less than magnitude to end point
			8. [x] Add actual step in
	*/

	// Buffers for later use
	Position buffer0;

	// Check if step should be taken
	Position abs_end_pos = _current_pos + relative_end_pos;
	if (relative_end_pos.magnitude() > .001 
		|| fabs(abs_end_pos.z) > Z_MAX_MAGNITUDE 
		|| fabs(abs_end_pos.roll) > ROLL_MAX_MAGNITUDE 
		|| fabs(abs_end_pos.pitch) > PITCH_MAX_MAGNITUDE) {
		
		// Check if move should be broken up into one or more steps
		if (fabs(_step_queue.getCurrentQueueEndPos().x + relative_end_pos.x) > X_MAX_NO_STEP_MAGNITUDE
			|| fabs(_step_queue.getCurrentQueueEndPos().y + relative_end_pos.y) > Y_MAX_NO_STEP_MAGNITUDE
			|| fabs(_step_queue.getCurrentQueueEndPos().yaw + relative_end_pos.yaw) > YAW_MAX_NO_STEP_MAGNITUDE) {

			uint32_t walk_time = 0;
			Position move_direction = relative_end_pos.unitVector();
			// Check if return to neutral is needed
			// If the step is in the opposite direction as the current end position, then the don't change step group 
			
			double max_step_magnitude_with_flip = _getMaxStepMagnitudeInDirection(move_direction, true);
			double max_step_magnitude_without_flip = _getMaxStepMagnitudeInDirection(move_direction, false);
			_Bool flip_first_step = max_step_magnitude_with_flip >= max_step_magnitude_without_flip ? true : false;
			double step_magnitude = flip_first_step ? max_step_magnitude_with_flip : max_step_magnitude_without_flip;
			_Bool returned_to_neutral = false;
			if (step_magnitude < get_max_step_magnitude()) { // If the biggest step in the right direction is less than the max step magnitude, then we need to return to neutral
				// Return to neutral IS needed
				// Enqueue a return to neutral move to move the legs back to the neutral position with out moving the body
				// Neutral position is 0,0,0 for X, Y, and Yaw; Z, Roll, and Pitch are unaffected
				buffer0.setPos(_step_queue.getCurrentQueueEndPos());
				buffer0.x = 0.00;
				buffer0.y = 0.00;
				buffer0.yaw = 0.00;
				walk_time += _step_queue.enqueue(buffer0, speed, StepType::RETURN_TO_NEUTRAL);
				returned_to_neutral = true;
			}

			Position traveled_pos;
			traveled_pos.clear();
			Position direction_vector = relative_end_pos.unitVector();
			
			// Step magnitude must be recalculated if the a return to neutral was performed
			// Otherise the calculation for checking if return to neutral was necessary will be sufficient
			if (returned_to_neutral) {
				step_magnitude = _getMaxStepMagnitudeInDirection(move_direction, false);
			}
			else {
				// Adjust step group if the first step should be flipped
				if (flip_first_step) {
					// Change step group for the first step
					_next_step_type = static_cast<decltype(_next_step_type)>(static_cast<uint8_t>(_next_step_type) ^ 1);
				}
			}

			// Check if move can be completed outright
			if (step_magnitude > relative_end_pos.magnitude()) {
				walk_time += _step_queue.enqueue(relative_end_pos, speed, _next_step_type);
				return walk_time; // The move is complete, exit the function
			}

			// Enqueue the first step in the direction of the move
			walk_time += _step_queue.enqueue(direction_vector * step_magnitude, speed, _next_step_type);
			traveled_pos += direction_vector * step_magnitude;

			// Enqueue steps until the end position is reached
			while (traveled_pos.magnitude() < relative_end_pos.magnitude()) {
				// Change step group between each step
				_next_step_type = static_cast<decltype(_next_step_type)>(static_cast<uint8_t>(_next_step_type) ^ 1);
				buffer0 = move_direction * _getMaxStepMagnitudeInDirection(move_direction, true);
				
				// Check if move can be completed outright
				if ((buffer0 + traveled_pos).magnitude() >= relative_end_pos.magnitude()) {
					// Complete the move
					walk_time += _step_queue.enqueue(relative_end_pos - traveled_pos, speed, _next_step_type);
					break; // The move is complete, exit the loop

				}
				else {
					walk_time += _step_queue.enqueue(buffer0, speed, _next_step_type);
					traveled_pos += buffer0;
				}

			}
			return walk_time;
		}
		else {
			return _step_queue.enqueue(relative_end_pos, speed, StepType::LINEAR_MOVE_RELATIVE);
		}
	}
	else {
		return 0;
	}
}

void Hexapod::wait(uint32_t time_ms) {
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		legs[i].wait(time_ms);
	} 
}

uint8_t Hexapod::legWaitSetup(uint8_t leg, uint32_t wait_time) {
	if (wait_time) {
		legs[leg].wait(wait_time);
		return 255;
	}
	return 0;
}

void Hexapod::returnToNeutral() {
	_return_to_neutral_flag = true;
}

uint8_t Hexapod::walkPerform() {
	if (_step_in_progress) {
		_Bool active_legs[NUM_LEGS];
		double step_progress = static_cast<double>(millis() - _move_start_time) / (_move_time);
		if (step_progress <= 1.0) {
			Position next_pos;
			uint8_t step_group;
			double adjusted_step_progress;
			switch(_current_step_type) {
				case StepType::LINEAR_MOVE_RELATIVE:
				case StepType::LINEAR_MOVE_ABSOLUTE:
					next_pos = (_end_pos - _start_pos) * step_progress + _start_pos;
					// All legs stay together and move to the same position
					for (uint8_t i = 0; i < NUM_LEGS; i++) {
						active_legs[i] = true;
					}					
					rapidMove(next_pos, active_legs);
					break;
				case StepType::RETURN_TO_NEUTRAL:
					// For first half of the step move one step group to neutral
					// For second half of the step move the other step group to neutral
					// Check if passing 0.5 step progress threshold or just starting, if so figure out which step group to move to neutral
					
					if ((_last_step_progress < 0.5 && step_progress >= 0.5) || (_last_step_progress >= 0.5 && step_progress < 0.5)) {
						// Switch step groups
						if (step_progress < 0.5) {
							switch(_last_step_type) {
								case StepType::GROUP0:
								case StepType::GROUP1:
									step_group = static_cast<uint8_t>(_last_step_type);
									break;
								default:
									step_group = 0;
									break;
							}
						}
						else {
							// Since return to neutral doesn't move the body, we reset the position when moving switching which legs are moving
							// to neutral as the opposite set of legs is in the reverse position as the other set of legs
							_start_pos.x *= -1.0;
							_start_pos.y *= -1.0;
							_start_pos.yaw *= -1.0;
							switch(_last_step_type) {
								case StepType::GROUP0:
								case StepType::GROUP1:
									step_group = static_cast<uint8_t>(_last_step_type) ^ 1;
									break;
								default:
									step_group = 1;
							}
						}
					}
					else {
						if (step_progress < 0.5) {
							switch(_last_step_type) {
								case StepType::GROUP0:
								case StepType::GROUP1:
									step_group = static_cast<uint8_t>(_last_step_type);
									break;
								default:
									step_group = 0;
									break;
							}
						}
						else {
							// Since return to neutral doesn't move the body, we reset the position when moving switching which legs are moving
							// to neutral as the opposite set of legs is in the reverse position as the other set of legs
							switch(_last_step_type) {
								case StepType::GROUP0:
								case StepType::GROUP1:
									step_group = static_cast<uint8_t>(_last_step_type) ^ 1;
									break;
								default:
									step_group = 1;
							}
						}
					}
					
					for (uint8_t i = 0; i < NUM_LEGS / NUM_STEP_GROUPS; i++) {
						active_legs[_step_groups[step_group][i]] = true;
						active_legs[_step_groups[(step_group^1)][i]] = false;
					}
					adjusted_step_progress = step_progress < 0.5 ? 2.0 * step_progress : (step_progress - 0.5) * 2.0;
					next_pos = (_end_pos - _start_pos) * adjusted_step_progress + _start_pos;
					next_pos.z -= -4 * adjusted_step_progress * (adjusted_step_progress - 1.0) * MAX_STEP_HEIGHT;
					rapidMove(next_pos, active_legs, true);
					break;
				case StepType::GROUP0:
				case StepType::GROUP1:
					next_pos = (_end_pos - _start_pos) * step_progress + _start_pos;
					step_group = static_cast<uint8_t>(_current_step_type);
					for (uint8_t i = 0; i < NUM_LEGS / NUM_STEP_GROUPS; i++) {
						active_legs[_step_groups[step_group][i]] = true;
						active_legs[_step_groups[(step_group^1)][i]] = false;
					}
					rapidMove(next_pos, active_legs);
					step_group ^= 1; 
					for (uint8_t i = 0; i < NUM_LEGS / 2; i++) {
						active_legs[_step_groups[step_group][i]] = true;
						active_legs[_step_groups[(step_group^1)][i]] = false;
					}
					next_pos.z -= -4 * step_progress * (step_progress - 1.0) * MAX_STEP_HEIGHT;
					next_pos.x *= -1.0;
					next_pos.y *= -1.0;
					next_pos.yaw *= -1.0;
					rapidMove(next_pos, active_legs, false);
					break;	
				case StepType::RAPID_MOVE:
					rapidMove(_end_pos);
					break;
			}
			_last_step_progress = step_progress;
			
			#if LOG_LEVEL >= CALCULATION_LOGGING 
				Serial.printf("Step progress: %f\n", step_progress);
				Serial.printf("Current position: x:%f, y:%f, z:%f, roll:%f, pitch:%f, yaw:%f\n",
				_current_pos.x, _current_pos.y, _current_pos.z, _current_pos.roll, _current_pos.pitch, _current_pos.yaw);
			#endif
		}
		else {
			_step_in_progress = false;
		}
		return 255;
	}
	else {
		if (_step_queue.isEmpty()) {
			// Nothing will be done if _walk_velocity magnitude is 0, otherwise a new step will be enqueued
			enqueueMaxStepInDirection(_walk_velocity, max(fabs(_walk_velocity.magnitude() / MAX_STEP_SPEED), 0.25));
		}
		else {
			_last_step_type = _current_step_type;
			_current_step_type = _step_queue.head->step_type;
			if (_current_step_type == StepType::GROUP0) {
				if (_last_step_type == StepType::GROUP1) {
					_current_pos.x *= -1.0;
					_current_pos.y *= -1.0;
					_current_pos.yaw *= -1.0;
				}
			}
			else if (_current_step_type == StepType::GROUP1) {
				if (_last_step_type == StepType::GROUP0) {
					_current_pos.x *= -1.0;
					_current_pos.y *= -1.0;
					_current_pos.yaw *= -1.0;
				}
			}
			
			switch(_current_step_type) {
				case StepType::RETURN_TO_NEUTRAL:
				case StepType::RAPID_MOVE:
				case StepType::LINEAR_MOVE_ABSOLUTE:
					_end_pos = _step_queue.head->end_pos;
					break;
				case StepType::GROUP0:
				case StepType::GROUP1:
				case StepType::LINEAR_MOVE_RELATIVE:
				default:
					_end_pos = _current_pos + _step_queue.head->end_pos;
					break;
			}
			_start_pos = _current_pos;
			_current_speed = _step_queue.head->speed;
			_move_time = _step_queue.head->time;
			_move_progress = 0.0;
			_move_start_time = millis();
			_step_queue.dequeue();
			_step_in_progress = true;
		}
	}
	return 255; // TODO: rethink return codes
}
uint16_t Hexapod::comboMovePerform() {
	// Return code is two 8 bit numbers
	// 8 grand bits are number of legs that got new end positions
	// 8 lesser bits are number of legs that are currently moving
	
	uint16_t retval = 0;
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if (legs[i].isMoving()) {
			retval += 1;
		}
		else {

			if (!_leg_queues[i].isEmpty()) {
				Operation * queue_head = _leg_queues[i].head;
				legWaitSetup(i, queue_head->wait_time_ms);
				if (!queue_head->wait_time_ms) {
					legLinearMoveSetup(i, queue_head->end_pos, queue_head->speed, queue_head->relative);
					#if LOG_LEVEL >= CALCULATION_LOGGING
						Serial.println("Leg " + String(i) + " moving to x:" + String(_leg_queues[i].head->end_pos.values[0]) + ", y: " + String(_leg_queues[i].head->end_pos.values[1]) + ", z: "
						+ String(_leg_queues[i].head->end_pos.values[2]) + "\nRelative: " + String(_leg_queues[i].head->relative) + " speed: " + String(_leg_queues[i].head->speed) + "\n");
					#endif
				}
				_leg_queues[i].dequeue();
				retval += (1 << 8);
			}
		}
	}
	linearMovePerform();
	return retval;
}

_Bool Hexapod::isBusy() {
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if (legs[i].isMoving())
			return true;
	}
	return _moving_flag || _high_level_move_flag;
}

_Bool Hexapod::isLowLevelBusy() {
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if (legs[i].isMoving())
			return true;
	}
	return _moving_flag;
}

uint8_t Hexapod::_inverseKinematics(double x, double y, double z, double roll, double pitch, double yaw, ThreeByOne *results) {
	Position pos;
	pos.set(x, y, z, roll, pitch, yaw);
	return _inverseKinematics(pos, results);
}

uint8_t Hexapod::_inverseKinematics(Position pos, ThreeByOne * results) {
	_Bool active_legs[NUM_LEGS];
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		active_legs[i] = true;
	} 
	return _inverseKinematics(pos, active_legs, results);
}

uint8_t Hexapod::_inverseKinematics(Position pos, _Bool active_legs[NUM_LEGS], ThreeByOne * results) {

	if (!_preCheckSafePos(pos))
        return 254; // Pre-check fail
	
	// Divide rotations by 100 to get radians (stored as hundredths of a radian to put on similar scale as x, y, z)
	pos.roll *= 0.01;
	pos.pitch *= 0.01;
	pos.yaw *= 0.01;

	double potential_results[NUM_LEGS][3];
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		potential_results[i][0] = pos.x;
		potential_results[i][1] = pos.y;
		potential_results[i][2] = pos.z + sin(pos.pitch) * (_leg_X_offset[i] + pos.x) + sin(pos.roll) * (_leg_Y_offset[i] + pos.y);
	}

	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		ThreeByOne temp = ThreeByOne(potential_results[i][0], potential_results[i][1], potential_results[i][2]);
		temp.rotateYaw(_home_yaws[i]);
		temp += _stance_offset;
		temp.rotateYaw(pos.yaw);
		for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
			potential_results[i][j] = temp.values[j];
		}
	}

	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if (!_postCheckSafeCoords(potential_results[i][0], potential_results[i][1], potential_results[i][2]))
			return 255; // Post-check fail
	}
	uint8_t results_index = 0;
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if(active_legs[i]) {
			results[results_index] = ThreeByOne(potential_results[i]);
			results_index++;
		}
		#if LOG_LEVEL >= CALCULATION_LOGGING
			Serial.println("IK value; leg: " + String(i) + " x:" + String(results[i].values[0]) + " y:" + String(results[i].values[1]) + " z:" + String(results[i].values[2]) + "\n");
		#endif
	}
	return 0;
}

_Bool Hexapod::_preCheckSafePos(Position pos) {
	// this function might be unnecessary but keeping as a placeholder
	//maybe dynamically set motor limits? check them here based on pos before moving. after movement complete calc new limits?
	return true;
}

_Bool Hexapod::_postCheckSafeCoords(double x, double y, double z) {
	// this function should check if legs will be in impossible positions or within the bot
	return true;
}

ThreeByOne Hexapod::forwardKinematics(double angle0, double angle1, double angle2) {
	ThreeByOne resulting_pos = legs[0].forwardKinematics(angle0, angle1, angle2);
	#if LOG_LEVEL >= CALCULATION_LOGGING
		Serial.println("FK value; x: " + String(resulting_pos.values[0]) + " y:" + String(resulting_pos.values[1]) + " z:" + String(resulting_pos.values[2]) + "\n");
	#endif
	return resulting_pos;
}

void Hexapod::runSpeed() {
	for (uint8_t i =0; i < NUM_LEGS; i++) {
		for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
			legs[i].axes[j].runSpeed();
		}
	}
}

double Hexapod::getDistance(Position target_pos) {
	double distance = 0;
	Position current_pos = _current_pos;
	double dx = current_pos.x - target_pos.x;
	double dy = current_pos.y - target_pos.y;
	distance = sqrt(dx * dx + dy * dy); // + dz * dz);
	return distance;
}

double Hexapod::getDistance(const Position& start_pos, const Position& end_pos) {
	double dx = start_pos.x - end_pos.x;
	double dy = start_pos.y - end_pos.y;
	return sqrt(dx * dx + dy * dy); // + dz * dz);
}

void Hexapod::legEnqueue(uint8_t leg, ThreeByOne op_end_pos, 
						 double op_speed, _Bool op_relative, uint32_t op_wait_time_ms) {
	_leg_queues[leg].enqueue(op_end_pos, op_speed, op_relative);
}

void Hexapod::legEnqueue(uint8_t leg, ThreeByOne op_end_pos, 
						 uint32_t op_time, _Bool op_relative, uint32_t op_wait_time_ms) {
	double op_speed;
	if (op_end_pos.magnitude() > 0.001) {
		if (!op_relative) {
			ThreeByOne current_queue_end_pos = _leg_queues[leg].getCurrentQueueEndPos();
			ThreeByOne distance_to_go = current_queue_end_pos - op_end_pos;
			op_speed = distance_to_go.magnitude() / (double(op_time) / 1000.00);
		}
		else {
			op_speed = op_end_pos.magnitude() / (double(op_time) / 1000.00);
		}
	}
	else {
		op_speed = 100.00;
	}
	legEnqueue(leg, op_end_pos, op_speed, op_relative, op_wait_time_ms);
}
