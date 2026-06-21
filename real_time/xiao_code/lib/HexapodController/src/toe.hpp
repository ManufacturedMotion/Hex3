#ifndef HEXA_TOE
#define HEXA_TOE

    #include <Arduino.h>
    #include <Wire.h>
    #include <Adafruit_VL6180X.h>

    class Toe
    {
        public:
            Toe();
            bool begin();
            void update();   //call regularly to update cached value, non-blocking
            float read();    // returns latest value
            bool isPressed();
            float toe_idle = 27.0f;
            float toe_threshold = 7.0f;
            float exposed_length = 47.0f;
        private:
            Adafruit_VL6180X sensor;
            // cached value (IMPORTANT: removes blocking reads)
            float _last_range = 0.0f;
            bool _valid = false;
    };

#endif