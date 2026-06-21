#ifndef HEXA_TOE
#define HEXA_TOE

    #include <Arduino.h>
    #include <Wire.h>
    #include <Adafruit_VL6180X.h>
    #include "config.hpp"
    #include "user_config.hpp"

    class Toe
    {
        public:
            Toe();
            bool begin();
            void update();   //call regularly to update cached value, non-blocking
            float read();    // returns latest value
            bool isPressed();
            float toe_idle = TOE_IDLE_READ[LEG_NUMBER]; 
            float toe_threshold = 0.075f; //percentage of remaining range to consider "pressed"
            float exposed_length = 47.0f;
        private:
            Adafruit_VL6180X sensor;
            // cached value (IMPORTANT: removes blocking reads)
            float _last_range = 0.0f;
            bool _valid = false;
            static constexpr uint8_t CALIB_SAMPLES = 30;
            float _calib_buffer[CALIB_SAMPLES] = {0};
            uint8_t _calib_index = 0;
            uint8_t _calib_count = 0;
            float _calib_sum = 0.0f;
        };

#endif