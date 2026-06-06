#include <Arduino.h>
#include <RP2040PIO_CAN.h>
#include <functional>

#ifndef HEX3_CAN
#define HEX3_CAN

class Leg;

#define CAN_PIO    0

class Can
{
    public:
        Can(
            uint8_t rx_pin,
            uint8_t tx_pin,
            CanBitRate bitrate,
            uint8_t leg_number,
            Leg* leg
        );
        bool begin();
        uint8_t handleCanMessage(const CanMsg& msg);
        void poll();
        std::function<void()> popPendingCommand();

    private:
        uint8_t _rx_pin;
        uint8_t _tx_pin;
        CanBitRate _bitrate;
        uint8_t _leg_number;
        uint32_t _node_id;
        Leg* _leg;
        void queueCommand(std::function<void()> fn);
        void handleCommandPayload(
            const uint8_t* d,
            uint16_t len
        );
        void sendIsoTp(
            const uint8_t* data,
            uint16_t len
        );
        void sendLegTelemetry();
        bool isFresh(uint32_t last);
};

#endif