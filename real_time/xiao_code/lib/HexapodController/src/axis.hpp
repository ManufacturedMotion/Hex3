#include <Arduino.h>
#include <stdint.h>
#include "mux.hpp"
#include <RP2040_PWM.h>
#include <PID_v1.h>

#ifndef HEX3_AXIS
#define HEX3_AXIS
    #define AXIS_POSITION_TRACK_INTERVAL_MS 1
    #define AXIS_POSITION_TOLERANCE 0.001 //rads
    #define AXIS_VELOCITY_TRACK_INTERVAL_MS 50

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
            void initializePositionLimits(double min_pos, double max_pos);
            uint8_t moveToPos();
            uint8_t moveToPos(double pos);
            uint8_t moveToPosAtSpeed(double pos, double target_speed);
			_Bool setMaxPos(double max_pos);
			_Bool setMinPos(double min_pos);
			double getCurrentPos();
			_Bool setMapping(double offset, double map_mult, _Bool reverse_axis);
			_Bool setMaxSpeed(double max_speed);
			double getMaxSpeed();
            double runSpeed();
			double getMaxPos();
			double getMinPos();
			void detach();
            void updatePos();
            void setSpeed(double speed);
            uint8_t setDutyCycle(bool dir, float duty_cycle);
            uint8_t setTargetPos(double pos);
            uint8_t setTargetVelocity(double velocity);
            uint8_t setTargetAcceleration(double acceleration);
            void trackMotion();
            double getCurrentVelocity();
            double getCurrentAcceleration();
            void allowMotion(bool allowed);
            void setControlConstants(double Kp, double Ki, double Kd, double Kv_ff, double Ka_ff);
            bool targetPosReached();
            float getDutyCycle();

        private:
            bool _allowed_to_move = true;
            uint8_t _pin_a = 0;
            uint8_t _pin_b = 0; 
            uint8_t _pin_c = 0;
            uint8_t _pin_d = 0;
            uint8_t _encoder_ch = 0;
            double _Kp;
            double _Ki;
            double _Kd;
            double _Kv_ff = 0.0; //0.0 unless otherwise specified
            double _Ka_ff = 0.0; //0.0 unless otherwise specified
            _Bool _4_pin = false;
            //_Bool _need_to_move = false;
            double _target_pos = NAN;
            double _target_velocity = 0.0;
            double _target_acceleration = 0.0;
            void _initializeAxis(); 
            Mux* _mux;
            const float POS_TOLERANCE = .1; //rad //TODO - lower me once encoder mounted
            double _max_speed = 10000;      //rad/s
            double _speed = 0.0;    //rad/s //TODO - fixme
			double _max_pos;        //rad
			double _min_pos;        //rad
			double _map_mult;       //unitless
			double _zero_pos;       //rad
			uint64_t _next_go_time; //milliseconds
			double _current_pos = 0.0; //rad
			uint16_t _move_time = 0; //milliseconds
			double _start_rads;
			double _end_rads;
			double _move_progress;
			uint32_t _move_start_time;
			_Bool _reverse_axis;
			double _axisMap(double x);
			double _radsToDegrees(double rads);
			double _degreesToRads(double degrees);
            RP2040_PWM* _pwm_instances[4]; //max 4 pins
            double _getCurrentPos();
            double _current_velocity = 0.0;
            double _current_acceleration = 0.0;
            double _last_position = 0.0;
            double _last_velocity = 0.0;
            uint32_t _last_pos_update_time = 0;
            uint32_t _last_vel_update_time = 0;
            double _control = 0.0;
            float _duty_cycle = 0;
            bool _dir = false;
            PID* _pid;
    };

#endif
