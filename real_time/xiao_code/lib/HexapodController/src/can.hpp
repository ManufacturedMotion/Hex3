#include <Arduino.h>
#include <RP2040PIO_CAN.h>

#ifndef HEX3_CAN
#define HEX3_CAN

#define CAN_PIO    0

class Can
{
    public:
        Can(uint8_t rx_pin, uint8_t tx_pin, CanBitRate bitrate, uint8_t leg_number);
        bool begin();
        void handleCanMessage(const CanMsg& msg);
        void poll();

    private:
        uint8_t _rx_pin;
        uint8_t _tx_pin;
        CanBitRate _bitrate;
        uint8_t _leg_number;
        uint32_t _node_id;
        void resetMTPS();
        void resetTune();
        bool isFresh(uint32_t last);
};

#endif