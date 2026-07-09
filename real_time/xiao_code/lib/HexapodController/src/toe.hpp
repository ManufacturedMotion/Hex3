#ifndef HEXA_TOE
#define HEXA_TOE



    #include <Arduino.h>
    #include <Wire.h>
    #include <Adafruit_VL6180X.h>
    #include "config.hpp"
    #include "user_config.hpp"

    #define TOE_MAX_UNCHANGED_COUNT 10 // number of consecutive readings that are unchanged before reinitializing sensor
    #define TOE_RESET_ACTION_DELAY_MS 50

    enum class ToeState {
        UNINITIALIZED,
        OPERATING,
        RESET_1,
        RESET_2,
        RESET_3,
        ERROR
    };
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
            ToeState state = ToeState::UNINITIALIZED;
            uint8_t medianFilter(uint8_t sample);

            
        private:
            Adafruit_VL6180X sensor;
            // cached value (IMPORTANT: removes blocking reads)
            float _last_range = 0.0f;
            bool _first_read = false; //for EMA filtering
            bool _connected = false;
            bool _initialized = false;
            static constexpr uint8_t CALIB_SAMPLES = 30;
            float _calib_buffer[CALIB_SAMPLES] = {0};
            uint8_t _calib_index = 0;
            uint8_t _calib_count = 0;
            float _calib_sum = 0.0f;
            bool probeSensor();
            uint32_t _last_probe_ms = 0;
            uint32_t _probe_interval_ms = 50;
            uint8_t _unchanged_count = 0;

            float _last_read_ms = 0;
            float _read_interval_ms = 10; //ms between reads
            uint8_t _median_buffer[3] = {0, 0, 0};
            uint8_t _median_index = 0;
            uint8_t _median_count = 0;
            uint32_t _last_action_time = 0;
        };

#endif