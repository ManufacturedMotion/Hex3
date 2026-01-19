#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>

#ifndef HEX3_MUX
#define HEX3_MUX

    #define MUX_ADDR 0x70  
    #define ENC_ADDR 0x36

    class Mux {
        public:
            Mux();
            void begin();
            void setChannel(uint8_t channel);
            double readEncoder(uint8_t channel);
        private:
            _Bool _initialized = false;
            uint8_t _last_channel = 255;

    };

#endif