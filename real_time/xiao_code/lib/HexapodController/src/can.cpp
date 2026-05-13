#include "can.hpp"
#include "hexapod.hpp"

//TODO - want to send all motor positions, torques periodically 
/* command types;
linear move
auto tune / home (TODO)
maybe quadratic move will be added
change old tune to SAM (single axis move)
rapid move for a single leg

ISO protocol for CAN open (multi frame messages, etc)


execution of commands;
  can returns an function object when command is ready
  main.cpp polls on hexapod.legs.can
  when can object is returning non-null, main.cpp executes the function object, which will execute the command and reset the can buffer to null
  when command not ready return null
*/

/*
CAN COMMAND FORMAT

All messages:
Byte 0  -> Command ID
Byte 1+ -> Payload (command-specific)

--------------------------------------------------
CMD_MOVE_MTPS (0x13)
--------------------------------------------------
Used for per-axis motion updates (buffered MTPS state)

Byte 0  -> 0x13 (CMD_MOVE_MTPS)
Byte 1  -> axis index (0..NUM_AXES_PER_LEG-1)
Byte 2-5 -> float position (4 bytes)

Total payload: 6 bytes

Notes:
- Stored per-axis in _mtps.leg_positions[axis]
- Overwrites previous value for that axis
- Execution is triggered elsewhere when buffer is complete/valid

--------------------------------------------------
CMD_MOVE_TUNE (0x14)
--------------------------------------------------
Used for single-axis tuning updates

Byte 0  -> 0x14 (CMD_MOVE_TUNE)
Byte 1  -> axis index
Byte 2-5 -> float position (4 bytes)

Total payload: 6 bytes

Notes:
- Stored in _tune.pos with associated axis
- Intended for fine adjustment / calibration use
*/


#if LOG_LEVEL >= BASIC_DEBUG
    #include <Arduino.h>
#endif

Can::Can(uint8_t rx_pin, uint8_t tx_pin, CanBitRate bitrate, uint8_t leg_number)
    : _rx_pin(rx_pin),
      _tx_pin(tx_pin),
      _bitrate(bitrate),
      _leg_number(leg_number),
      _node_id(0x100 + leg_number)
{}

bool Can::begin()
{
    CAN.setRX(_rx_pin);
    CAN.setTX(_tx_pin);

    if (!CAN.begin(_bitrate)) {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("CAN init failed");
        #endif
        return false;
    }

    #if LOG_LEVEL >= BASIC_DEBUG
        Serial.printf("CAN ready | node: 0x%X\n", _node_id);
    #endif

    return true;
}

enum CommandSubtype : uint8_t
{
    CMD_MOVE_MTPS    = 0x13,
    CMD_MOVE_TUNE    = 0x14
};

struct MTPSBuffer
{
    bool valid = false;
    uint32_t last_update = 0;
    float leg_positions[NUM_AXES_PER_LEG];
    float speed;
};

struct TuneBuffer
{
    bool valid = false;
    uint32_t last_update = 0;
    uint8_t axis;
    float pos;
};

static MTPSBuffer _mtps;
static TuneBuffer _tune;

static const uint32_t CMD_TIMEOUT_MS = 250;

bool Can::isFresh(uint32_t last)
{
    return (millis() - last) <= CMD_TIMEOUT_MS;
}

void Can::resetMTPS()
{
    _mtps.valid = false;

    #if LOG_LEVEL >= BASIC_DEBUG
        Serial.println("CAN: MTPS reset");
    #endif
}

void Can::resetTune()
{
    _tune.valid = false;

    #if LOG_LEVEL >= BASIC_DEBUG
        Serial.println("CAN: TUNE reset");
    #endif
}

void Can::handleCanMessage(const CanMsg& msg)
{
    const uint8_t* d = msg.data;
    uint8_t cmd = d[0];
    uint32_t now = millis();

    if (msg.id != _node_id)
    {
        return; 
    }

    switch (cmd)
    {
        default:
        {
            Serial.println("CAN: Unsupported command received");
            Serial.println(cmd, HEX);
            return;
        }   

        case CMD_MOVE_MTPS:
        {
            _mtps.last_update = now;
            uint8_t axis = d[1];
            float pos;

            memcpy(&pos, &d[2], sizeof(float));

            if (axis < NUM_AXES_PER_LEG)
            {
                _mtps.leg_positions[axis] = pos;
                _mtps.valid = true;
                Serial.printf("CAN MTPS exec | axis %d | pos %.3f\n", axis, pos);
            }
            return;
        }

        case CMD_MOVE_TUNE:
        {
            _tune.last_update = now;
            _tune.axis = d[1];
            memcpy(&_tune.pos, &d[2], sizeof(float));
            _tune.valid = true;
            //TODO - add executor
            Serial.printf("CAN TUNE exec | axis %d | pos %.3f\n", _tune.axis, _tune.pos);

            return;
        }
    }
}

void Can::poll()
{
    uint32_t now = millis();

    if (_mtps.valid && !isFresh(_mtps.last_update))
    {
        resetMTPS();
    }

    if (_tune.valid && !isFresh(_tune.last_update))
    {
        resetTune();
    }
}