#include "axis.hpp"
#include "leg.hpp"
#include "position.hpp"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "operation_queue.hpp"
#include "three_by_matrices.hpp"
#include "voltage_monitor.hpp"
#include "step_queue.hpp"

#ifndef HEXA_H
#define HEXA_H

	#define NUM_LEGS 6
	#define NUM_STEP_GROUPS 2
	#define MAX_STEP_MAGNITUDE 75.0
	#define STEP_TO_NEUTRAL_SPEED 200.0
	
	// TODO: Confirm and refine limits
	#define Z_MAX_MAGNITUDE 200.0
	#define ROLL_MAX_MAGNITUDE (M_PI / 8.0)
	#define PITCH_MAX_MAGNITUDE (M_PI / 8.0)
	#define X_MAX_NO_STEP_MAGNITUDE 20.0
	#define Y_MAX_NO_STEP_MAGNITUDE 20.0
	#define YAW_MAX_NO_STEP_MAGNITUDE (M_PI / 32.0)
	#define MAX_STEP_HEIGHT 50.0

	#define MAX_STEP_SPEED 300.0

	class Hexapod {
		public:
			Hexapod();
			void moveLegAxisToPos(uint8_t leg_number, uint8_t axis_number, double target_position);
			_Bool moveLegToPos(uint8_t leg_number, double x, double y, double z);
			void moveToZeros();
			void sit();
			void stand();
			void wait(uint32_t time_ms);
			Leg legs[NUM_LEGS];
			ThreeByOne forwardKinematics(double angle0, double angle1, double angle2);
			void rapidMove(double x, double y, double z, double roll, double pitch, double yaw);
			uint8_t linearMoveSetup(double x, double y, double z, double roll, double pitch, double yaw, double target_speed);
			uint8_t linearMoveSetup(Position next_pos, double target_speed);
			uint8_t linearMoveSetup(double x, double y, double z, double roll, double pitch, double yaw, double target_speed, _Bool active_legs[NUM_LEGS]);
			uint8_t linearMoveSetup(Position next_pos, double target_speed, _Bool active_legs[NUM_LEGS]);
			uint8_t legLinearMoveSetup(uint8_t leg, double x,  double y, double z, double target_speed, _Bool relative = false);
			uint8_t legLinearMoveSetup(uint8_t leg, ThreeByOne end_coord, double target_speed, _Bool relative = false);
			void linearMovePerform();
			void rapidMove(Position next_pos);
			void rapidMove(Position next_pos, _Bool active_legs[NUM_LEGS], _Bool update_current_pos=true);
			void runSpeed();
			_Bool isBusy();
      		double getDistance(Position target_pos);
			double getDistance(const Position& start_pos, const Position& end_pos);
			_Bool isLowLevelBusy();
			uint8_t stepSetup(double x, double y, double z, double speed);
			uint8_t stepSetup(ThreeByOne relative_end_coord, double speed);
			uint8_t stepToNeutral(double speed);
			uint8_t enqueueRapidMove(Position pos);

			uint8_t walkSetup(ThreeByOne relative_end_coord, double speed, _Bool return_to_neutral = false);
			uint32_t walkSetup(Position relative_end_pos, double speed);

			uint8_t walkSetup(double x, double y, double speed, _Bool return_to_neutral = false);
			uint8_t walkPerform();
			uint16_t comboMovePerform();
			void opQueueTest();
			double get_max_step_magnitude();
			uint8_t legWaitSetup(uint8_t leg, uint32_t wait_time);
			void returnToNeutral();
			ThreeByOne get_max_step(ThreeByOne);
			void legEnqueue(uint8_t leg, ThreeByOne op_end_pos, double op_speed, _Bool op_relative, uint32_t op_wait_time_ms = 0);
			void legEnqueue(uint8_t leg, ThreeByOne op_end_pos, uint32_t op_time, _Bool op_relative, uint32_t op_wait_time_ms = 0);
			void startUp();
			void setWalkVelocity(Position velocity);
			uint32_t enqueueMaxStepInDirection(Position direction_vector, double scalar);
			void dance0();
			void dance1();
			void dance2();
			void dance3();
			void dance4();
			void dance5();
			uint32_t enqueueLinearMove(Position next_pos, double target_speed, bool relative=false);
			void detachAllServos();



		private:
			double _last_step_progress;
			double _current_speed = 100.0;
			StepType _current_step_type;
			StepType _last_step_type;
			_Bool _step_in_progress = false;
			uint8_t _step_groups[NUM_STEP_GROUPS][NUM_LEGS / 2] = {{0,2,4}, {1,3,5}}; // Divide legs into two self-stable groups
			uint8_t _next_step_group = 0;
			_Bool _linear_move_legs[6] = {false, false, false, false, false, false};
			double _next_leg_pos[NUM_LEGS][NUM_AXES_PER_LEG];
			double _leg_X_offset[NUM_LEGS] = {-54.608, -109.215, -54.608,  54.608, 109.215, 54.608};
			double _leg_Y_offset[NUM_LEGS] = { 94.583,    0.000, -94.583, -94.583,   0.000, 94.583};
			double _home_yaws[NUM_LEGS] = { ((-M_PI / 2.0) - 1.0),	(-M_PI / 2.0),	((-M_PI / 2.0) + 1.0), 
											(1.0), 					( M_PI / 2.0),	(( M_PI / 2.0) + 1.0)};
			ThreeByOne _stance_offset = ThreeByOne(0.0, 100.0, 0.0);
			// double _home_yaws[NUM_LEGS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			double _max_speed = 1000000;
			_Bool _preCheckSafePos(Position pos);
			_Bool _postCheckSafeCoords(double x, double y, double z);
			uint8_t _inverseKinematics(Position pos, ThreeByOne * results);
			uint8_t _inverseKinematics(Position pos, _Bool active_legs[NUM_LEGS], ThreeByOne * results);
			uint8_t _inverseKinematics(double x, double y, double z, double roll, double pitch, double yaw, ThreeByOne * results);
			void _moveLegs();
			double _move_progress;
			uint32_t _move_start_time;
			_Bool _moving_flag = false;
			_Bool _high_level_move_flag = false;
			_Bool _return_to_neutral_flag = false;
			_Bool _neutral_position_flag = false;
			Position _end_pos;
			Position _start_pos;
			Position _current_pos;
			Position _walk_velocity;
			double _move_time;
			OperationQueue _leg_queues[NUM_LEGS];
			ThreeByOne _current_step_permutation[NUM_STEP_GROUPS];
			ThreeByOne _previous_step_unit_vector = ThreeByOne(0.0, 0.0, 0.0);
			StepQueue _step_queue;
			StepType _next_step_type = StepType::GROUP0;
			void setMoveLegs(StepType step_type, _Bool * active_legs[NUM_LEGS]);
			double _getMaxStepMagnitudeInDirection(Position direction_vector, _Bool flipped_step_group);

};

#endif
