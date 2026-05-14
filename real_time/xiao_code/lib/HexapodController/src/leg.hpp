#include "axis.hpp"
#include "config.hpp"
#include "three_by_matrices.hpp"
#include "mux.hpp"
#include "voltage_monitor.hpp"
#include <stdbool.h>
#include <stdint.h>

#define LEG_DEBUG

#ifndef HEXA_LEG
#define HEXA_LEG

	class Can;

	#define NUM_AXES_PER_LEG 3
	#define LINEAR_MOVE_INTERVAL_MS 6
	#define LEG_POSITION_TRACK_INTERVAL_MS 6
	#define LEG_VELOCITY_TRACK_INTERVAL_MS 30
	#define MAX_LINEAR_ACCELERATION 500.0 //mm/s^2

	enum move_stage {ACCELERATING, CRUISING, DECELERATING, STOPPED, UNINITIALIZED};

	#define TOE_PIN D0
	class Leg {
		public:
			Leg();
			Can* can;
			void initializeAxes(uint8_t leg_number);
			double current_angles[NUM_AXES_PER_LEG];
			_Bool rapidMove(double x,  double y, double z);
			Axis axes[NUM_AXES_PER_LEG];
			ThreeByOne getCurrentPosition();
			ThreeByOne getEndPosition();
			ThreeByOne forwardKinematics(double axis0_angle, double axis1_angle, double axis2_angle);
			_Bool linearMoveSetup(double x,  double y, double z, double target_speed, _Bool relative = false);
			uint8_t linearMovePerform();
			_Bool isMoving();
			void wait(uint32_t time_ms);
			void begin();
			Mux mux;
			_Bool toePressed();
			void runSpeed();
			void setAxisDutyCycle(uint8_t axis_number, bool dir, float duty_cycle);
			void setAxisTargetPos(uint8_t axis_number, double pos);
			void stopAxis(uint8_t axis_number);
			void setAxisControlConstants(uint8_t axis_number, double tuning_voltage, double Kp_pos, double Kd_pos, double Kp_vel, double Ki_vel, double Kv_ff);
			_Bool rapidMove(ThreeByOne target_pos);
			VoltageSensor voltage_sensor = VoltageSensor();
			void setAxisTargetVelocity(uint8_t axis_number, double target_velocity);

		private:
			uint8_t _leg_number;
			double _length0 = 112.929;
			double _length1 = 96.00;
			double _length2 = 150.5;
			uint8_t toe_threshold = 100;
			void _moveAxes();
			_Bool _checkSafeCoords(double x, double y, double z);
			_Bool _inverseKinematics(double x, double y, double z);
			_Bool _forwardKinematics(double theta0, double theta1, double theta2, double& x, double& y, double& z);
			void _trackMotion();

			uint32_t _last_pos_update_time = 0;
			uint32_t _last_velocity_update_time = 0;
			double _next_angles[NUM_AXES_PER_LEG];
			double _current_angles[NUM_AXES_PER_LEG];
			double _next_velocities[NUM_AXES_PER_LEG];
			double _current_velocities[NUM_AXES_PER_LEG];
			uint32_t _last_linear_move_time = 0;

			double _current_cartesian[NUM_AXES_PER_LEG];
			
		
			// Position tracking variables
			uint32_t _last_position_track_time;
			double _current_pos[NUM_AXES_PER_LEG];
			double _current_velocity[NUM_AXES_PER_LEG];
			double _current_acceleration[NUM_AXES_PER_LEG];
			double _last_pos[NUM_AXES_PER_LEG];
			double _last_velocity[NUM_AXES_PER_LEG];

			double _next_cartesian[NUM_AXES_PER_LEG];
			double _start_cartesian[NUM_AXES_PER_LEG];
			double _end_cartesian[NUM_AXES_PER_LEG];
			uint32_t _move_start_time; 
			double _max_speed = 1000000.0;
			uint32_t _move_time;
			_Bool _moving_flag = false;
			void * _movement_function;
			uint8_t _volage_pin; //TODO get this value
			double _target_speed;
			double _last_speed;
			uint32_t _accel_time;
			ThreeByOne _direction_vector;
			
			move_stage _move_stage = move_stage::UNINITIALIZED;

	};

#endif
