/**
 * @file leg.hpp
 * @brief Hexapod leg controller class definition
 *
 * Provides control for a single leg of the hexapod robot with three axes (joints).
 * Implements inverse/forward kinematics, motion tracking, and linear movement with
 * acceleration profiles.
 */

#include "axis.hpp"
#include "config.hpp"
#include "three_by_matrices.hpp"
#include "mux.hpp"
#include "voltage_monitor.hpp"
#include "command_queue.hpp"
#include "toe.hpp"
#include <stdbool.h>
#include <stdint.h>

#define LEG_DEBUG

#ifndef HEXA_LEG
#define HEXA_LEG
	#define NUM_LEGS 6                             ///< Total number of legs in the hexapod
	#define NUM_AXES_PER_LEG 3                     ///< Number of joints per leg
	#define LINEAR_MOVE_INTERVAL_MS 6              ///< Update interval for linear movement (ms)
	#define LEG_POSITION_TRACK_INTERVAL_MS 6       ///< Position tracking update interval (ms)
	#define LEG_VELOCITY_TRACK_INTERVAL_MS 30      ///< Velocity/acceleration tracking interval (ms)
	#define MAX_LINEAR_ACCELERATION 500.0          ///< Maximum linear acceleration (mm/s^2)
	#define TOE_UPDATE_INTERVAL_MS 30                ///< Minimum interval between toe sensor updates (ms)

	class Can;
	enum move_stage {ACCELERATING, CRUISING, DECELERATING, STOPPED, UNINITIALIZED};

	/**
	 * @class Leg
	 * @brief Represents a single leg of the hexapod robot
	 *
	 * Controls three joint axes and provides kinematics calculations and motion planning.
	 * Supports both rapid (instantaneous) and linear (velocity-profiled) movements.
	 */
	class Leg {
		public:
			Leg();
			Can* can;
			void initializeAxes(uint8_t leg_number);
			double current_angles[NUM_AXES_PER_LEG];
			_Bool rapidMove(double x, double y, double z);
			Axis axes[NUM_AXES_PER_LEG];
			_Bool linearMoveSetup(double x, double y, double z, double target_speed, _Bool relative = false);
			uint8_t linearMovePerform();
			void begin();
			Mux mux;
			CommandQueue command_queue;
			/// Main control loop - runs PID, logs telemetry, updates kinematics
			void runSpeed();
			void setAxisTargetPos(uint8_t axis_number, double pos);
			void stopAxis(uint8_t axis_number);
			void setAxisControlConstants(uint8_t axis_number, double Kp_pos, double Kd_pos, double Kp_vel, double Ki_vel, double Kv_ff);
			_Bool rapidMove(ThreeByOne target_pos);
			VoltageSensor voltage_sensor = VoltageSensor();
			void processCommandQueue();
			Toe toe;
			float readToe();
		private:
			// Physical properties and calibration
			uint8_t _leg_number;                         ///< Identifier for this leg (0-5)
			double _length0 = 112.929;                   ///< Length of base link (mm)
			double _length1 = 96.00;                     ///< Length of first joint link (mm)
			double _length2 = 196.55; 		   			 ///< Length of second joint link (mm) 
			double _length2_dynamic = _length2;             ///< Length of second joint link (mm) with adjustement for toe compression

			float _last_compression_distance = 0.0f; ///< Last measured compression distance of the toe sensor (mm)
			uint32_t _last_toe_update_time = 0;      ///< Timestamp of last toe update
			void _updateToe();
			float _toe_value = 0.0f; ///< Cached toe sensor value (mm)
			
			// Kinematics calculation methods
			/// Move all axes to target angles calculated by inverse kinematics
			void _moveAxes();
			
			/// Validate that Cartesian coordinates are safe for the leg
			_Bool _checkSafeCoords(double x, double y, double z);
			
			/// Calculate joint angles from Cartesian position
			_Bool _inverseKinematics(double x, double y, double z);
			
			/// Calculate Cartesian position from joint angles
			_Bool _forwardKinematics(double theta0, double theta1, double theta2, double& x, double& y, double& z);
			
			/// Update position and velocity tracking from current axis positions
			void _trackMotion();

			// Motion tracking variables
			uint32_t _last_pos_update_time = 0;          ///< Timestamp of last position update
			uint32_t _last_velocity_update_time = 0;     ///< Timestamp of last velocity update
			
			// Joint angle tracking (radians)
			double _next_angles[NUM_AXES_PER_LEG];       ///< Target angles for next move
			double _current_angles[NUM_AXES_PER_LEG];    ///< Current measured angles
			double _next_velocities[NUM_AXES_PER_LEG];   ///< Target velocities
			double _current_velocities[NUM_AXES_PER_LEG];///< Current measured velocities (rad/s)
			uint32_t _last_linear_move_time = 0;         ///< Timestamp of last linear move update
			
			// Cartesian position tracking
			double _current_cartesian[NUM_AXES_PER_LEG]; ///< Current XYZ position (mm)
			
			// Position derivatives for motion analysis
			double _current_pos[NUM_AXES_PER_LEG];       ///< Cartesian position (mm)
			double _current_velocity[NUM_AXES_PER_LEG];  ///< Cartesian velocity (mm/s)
			double _current_acceleration[NUM_AXES_PER_LEG]; ///< Cartesian acceleration (mm/s^2)
			double _last_pos[NUM_AXES_PER_LEG];          ///< Previous position for derivative
			double _last_velocity[NUM_AXES_PER_LEG];     ///< Previous velocity for derivative
			
			// Linear movement variables (for velocity-profiled moves)
			double _next_cartesian[NUM_AXES_PER_LEG];    ///< Next setpoint during linear move
			double _start_cartesian[NUM_AXES_PER_LEG];   ///< Starting position of move
			double _end_cartesian[NUM_AXES_PER_LEG];     ///< Target position of move
			uint32_t _move_start_time;                   ///< Timestamp when move started
			double _max_speed = 1000000.0;               ///< Maximum allowable speed (mm/s)
			uint32_t _move_time;                         ///< Total time for move (ms)
			_Bool _moving_flag = false;                  ///< Whether a move is in progress
			double _target_speed;                        ///< Current target speed (mm/s)
			double _last_speed;                          ///< Previous speed for smoothing
			uint32_t _accel_time;                        ///< Acceleration time (ms)
			ThreeByOne _direction_vector;                ///< Unit vector direction of motion
			
			/// Current stage of multi-phase movement
			move_stage _move_stage = move_stage::UNINITIALIZED;

	};

#endif
