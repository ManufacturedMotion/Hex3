#ifndef HEXA_TOE
#define HEXA_TOE

    #define TOE_PIN D0  ///< Pin for toe/ground contact sensor gpio
    #include <Adafruit_VL6180X.h>

    class Toe {
        public:
            Toe(bool useGPIO = false);
            bool begin();
            float read();
            float toe_idle = 27.0; //toe resting reading bounces between 25-27
            float toe_threshold = 7.0; //if reading is below we can assume fully compressed
            float exposed_length = 47.0; //how much the toe extends beyond the end of the leg when uncompressed (mm) //TODO double check this in CAD
        private:
            Adafruit_VL6180X sensor;
            bool gpioEnabled;
    };

#endif