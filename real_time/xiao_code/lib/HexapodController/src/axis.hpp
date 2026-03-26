#include <Arduino.h>
#include <stdint.h>
#include "mux.hpp"
#include <RP2040_PWM.h>
#include <PID_v1.h>

#ifndef HEX3_AXIS
#define HEX3_AXIS
    #define AXIS_POSITION_TRACK_INTERVAL_MS 1
    #define AXIS_POSITION_TOLERANCE 0.001 //rads
    #define AXIS_VELOCITY_TRACK_INTERVAL_MS 1
    #define MOMENTUM_MONITOR_INTERVAL_MS 5
    #define DISTURBANCE_MONITOR_INTERVAL_MS 10

    //pindefs for use in leg.cpp
    //S1: 11 12;            ch 5 (?)
    //S2A: 18 2; S2B: 17 3; ch 6 (?)
    //S3: 16 15;            ch 7 (?)

    //TODO - pwm tuning
    //I am treating motor as 'joint'. so S2 has both motors, hence 4 pins. figure it is better because S2 only has one encoder
    class Axis {
        public:
            Axis();
            void link(uint8_t pin_a, uint8_t pin_b, uint8_t encoder_ch, Mux& mux_ref);
            void link(uint8_t pin_a, uint8_t pin_b, uint8_t pin_c, uint8_t pin_d, uint8_t encoder_ch, Mux& mux_ref);
            void stopAxis();
            void initializePositionLimits(float min_pos, float max_pos);
            uint8_t moveToPos();
            uint8_t moveToPos(float pos);
            uint8_t moveToPosAtSpeed(float pos, float target_speed);
			_Bool setMaxPos(float max_pos);
			_Bool setMinPos(float min_pos);
			float getCurrentPos();
			_Bool setMapping(float offset, float map_mult, _Bool reverse_axis);
			_Bool setMaxSpeed(float max_speed);
			float getMaxSpeed();
            float runSpeed();
			float getMaxPos();
			float getMinPos();
			void detach();
            void updatePos();
            void setSpeed(float speed);
            uint8_t setDutyCycle(bool dir, float duty_cycle);
            uint8_t setTargetPos(float pos);
            uint8_t setTargetVelocity(float velocity);
            uint8_t setTargetAcceleration(float acceleration);
            void trackMotion();
            float getCurrentVelocity();
            float getCurrentAcceleration();
            void allowMotion(bool allowed);
            void setControlConstants(float tuning_voltage, float Kp_pos, float Kd_pos, float Kp_vel, float Ki_vel, float Kv_ff);
            bool targetPosReached();
            float getDutyCycle();
            float getCorrectedDutyCycle();
            float getEstimatedTorque();
            uint8_t moveAtVelocity();
            uint8_t setFeedforwardVelocity(float velocity);
            void momentumMonitor();
            void disturanceMonitor();
            void setInputVoltage(float voltage);
            float getInputVoltage();
            
            float getMOBDisturbanceTorque();

        private:
            void _updateMotorCurrentEstimate();
            float _torqueToDutyCycle(float torque);

            bool _allowed_to_move = true;
            uint8_t _pin_a = 0;
            uint8_t _pin_b = 0; 
            uint8_t _pin_c = 0;
            uint8_t _pin_d = 0;
            uint8_t _encoder_ch = 0;
            float _tuning_voltage;
            float _input_voltage = -1.0; //default to impossible number to indicate not set
            double _Kp_pos;
            double _Ki_pos;
            double _Kd_pos;
            double _Kp_vel;
            double _Ki_vel;
            double _Kd_vel;
            double _Kv_ff = 0.0; //0.0 unless otherwise specified
            // float _Ka_ff = 0.0; //0.0 unless otherwise specified
            _Bool _4_pin = false;
            //_Bool _need_to_move = false;
            double _target_pos = NAN;
            double _target_velocity = 0.0;
            double _target_acceleration = 0.0;
            void _initializeAxis(); 
            Mux* _mux;
            const float POS_TOLERANCE = .1; //rad //TODO - lower me once encoder mounted
            float _max_speed = 10000;      //rad/s
            float _speed = 0.0;    //rad/s //TODO - fixme
			float _max_pos;        //rad
			float _min_pos;        //rad
			float _map_mult;       //unitless
			float _zero_pos;       //rad
			uint64_t _next_go_time; //milliseconds
			double _current_pos = 0.0; //rad
			uint16_t _move_time = 0; //milliseconds
			float _start_rads;
			float _end_rads;
			float _move_progress;
			uint32_t _move_start_time;
			_Bool _reverse_axis;
			float _axisMap(float x);
			float _radsToDegrees(float rads);
			float _degreesToRads(float degrees);
            RP2040_PWM* _pwm_instances[4]; //max 4 pins
            float _getCurrentPos();
            double _current_velocity = 0.0;
            float _current_acceleration = 0.0;
            float _last_position = 0.0;
            float _last_velocity = 0.0;
            uint32_t _last_pos_update_time = 0;
            uint32_t _last_vel_update_time = 0;
            double _pos_control = 0.0;
            double _vel_control = 0.0;
            float _duty_cycle = 0;
            bool _dir = false;
            float _back_EMF_constant = 5.0; // rad /V //1.0 / 17.66819; // 3978.876 RPM / V / 56.3 * 4.0, Rough estimate based on motor specs and gear ratio
            float _torque_constant = _back_EMF_constant;//8.27 / ((1.0 / _back_EMF_constant) * (.10472)); // Nm/A, derived from stall torque and stall current, adjusted for gear ratio
            float _resistance = 7.5; // Ohms, rough estimate based on motor specs
            float _estimated_current = 0.0; // Amps
            float _estimated_torque = 0.0; //Nm
            float _min_duty = 52.5; //minimum duty cycle to overcome motor deadzone from standstill
            PID* _pid_pos;
            PID* _pid_vel;
            float _feedforward_velocity = 0.0;

            float _total_inertia = 2.0; //kg*m^2, very rough estimate for now, will be used for disturbance monitoring and feedforward acceleration control once implemented
            float _MOB_disturbance_torque = 0.0; //momentum observer disturbance torque estimate
            float _DOB_disturbance_torque = 0.0; //disturbance observer disturbance torque estimate

            uint32_t _last_momentum_monitor_update_time = 0;
            uint32_t _last_disturbance_monitor_update_time = 0;
            float _momentum_monitor_gain = 20.0; //tuning parameter for momentum observer, higher means more aggressive disturbance compensation but also more noise sensitivity
            float _disturbance_monitor_gain = 0.001; //tuning parameter for disturbance observer, higher means more aggressive disturbance compensation but also more noise sensitivity
            float _momentum_hat = 0.0; //internal variable for momentum observer
        };

#endif
