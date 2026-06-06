#include "can.hpp"
#include "leg.hpp"
#include "log_levels.hpp"
#include <functional>

/*

execution of commands;
  can returns an function object when command is ready
  main.cpp polls on hexapod.legs.can
  when can object is returning non-null, main.cpp executes the function object, which will execute the command and reset the can buffer to null
  when command not ready return null

  how it looks for main
    leg.can->poll();

    auto fn = leg.can->popPendingCommand();

    if (fn)
    {
        fn();
    }

TODO:
- currently only one pending command can exist at a time
- may need to scale to a queue / ring buffer of commands
- especially important for high frequency motion updates
*/

/*
CAN COMMAND FORMAT

All messages:
Byte 0  -> Command ID
Byte 1+ -> Payload (command-specific)

--------------------------------------------------
CMD_LINEAR_MOVE (0x10)
--------------------------------------------------
Multi-axis coordinated move

Payload:
Byte 0      -> command id
Byte 1      -> move time ms
Byte 2..n   -> axis float positions

Typically ISO-TP multi-frame

--------------------------------------------------
CMD_AUTO_TUNE (0x11)
--------------------------------------------------
Auto calibration / homing command

Payload:
Byte 0 -> command id

Single frame

--------------------------------------------------
CMD_QUADRATIC_MOVE (0x12)
--------------------------------------------------
Quadratic trajectory move

Payload:
Byte 0      -> command id
Byte 1..n   -> trajectory params

Typically ISO-TP multi-frame

--------------------------------------------------
CMD_SINGLE_AXIS_MOVE (0x13)
--------------------------------------------------
Move one axis only

Payload:
Byte 0      -> command id
Byte 1      -> axis
Byte 2..5   -> float position

Single frame

--------------------------------------------------
CMD_RAPID_MOVE (0x14)
--------------------------------------------------
Rapid move for one leg

Payload:
Byte 0      -> command id
Byte 1..n   -> rapid move params

Can be single or multi-frame

--------------------------------------------------
CMD_LEG_STATE (0x20)
--------------------------------------------------
Periodic leg telemetry message

Payload:
Byte 0       -> command id
Byte 1..12   -> 3 float positions
Byte 13..24  -> 3 float torques

Always ISO-TP multi-frame

Notes:
- sent periodically from leg -> host
*/

Can::Can(
    uint8_t rx_pin,
    uint8_t tx_pin,
    CanBitRate bitrate,
    uint8_t leg_number,
    Leg* leg
)
    : _rx_pin(rx_pin),
      _tx_pin(tx_pin),
      _bitrate(bitrate),
      _leg_number(leg_number),
      _node_id(0x100 + leg_number),
      _leg(leg)
{}

bool Can::begin()
{
    CAN.setRX(_rx_pin);
    CAN.setTX(_tx_pin);

    if (!CAN.begin(_bitrate, 64))
    {
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
    CMD_LINEAR_MOVE       = 0x10,
    CMD_AUTO_TUNE         = 0x11,
    CMD_QUADRATIC_MOVE    = 0x12,
    CMD_SINGLE_AXIS_MOVE  = 0x13,
    CMD_RAPID_MOVE        = 0x14,

    CMD_LEG_STATE         = 0x20
};

enum IsoTpFrameType : uint8_t
{
    ISO_TP_SINGLE_FRAME      = 0x0,
    ISO_TP_FIRST_FRAME       = 0x1,
    ISO_TP_CONSECUTIVE_FRAME = 0x2,
    ISO_TP_FLOW_CONTROL      = 0x3
};

struct IsoTpRxBuffer
{
    bool active = false;

    uint32_t last_update = 0;

    uint16_t expected_size = 0;
    uint16_t current_size = 0;

    uint8_t sequence_number = 1;

    uint8_t data[256];
};

static IsoTpRxBuffer _isotp_rx;

static const uint32_t CMD_TIMEOUT_MS = 250;
static const uint32_t TELEMETRY_INTERVAL_MS = 100;

static uint32_t _last_telemetry_tx = 0;

static std::function<void()> _pending_command = nullptr;

bool Can::isFresh(uint32_t last)
{
    return (millis() - last) <= CMD_TIMEOUT_MS;
}

static void resetIsoTp()
{
    _isotp_rx.active = false;

    _isotp_rx.expected_size = 0;
    _isotp_rx.current_size = 0;

    _isotp_rx.sequence_number = 1;

    #if LOG_LEVEL >= BASIC_DEBUG
        Serial.println("CAN: ISO-TP reset");
    #endif
}

std::function<void()> Can::popPendingCommand()
{
    std::function<void()> fn = _pending_command;

    _pending_command = nullptr;

    return fn;
}

void Can::queueCommand(std::function<void()> fn)
{
    _pending_command = fn;
}

void Can::sendIsoTp(const uint8_t* data, uint16_t len)
{
    if (len <= 7)
    {
        CanMsg tx{};

        tx.id = _node_id;

        tx.data[0] =
            (ISO_TP_SINGLE_FRAME << 4) |
            (len & 0x0F);

        memcpy(&tx.data[1], data, len);

        tx.data_length = len + 1;

        CAN.write(tx);

        return;
    }

    {
        CanMsg tx{};

        tx.id = _node_id;

        tx.data[0] =
            (ISO_TP_FIRST_FRAME << 4) |
            ((len >> 8) & 0x0F);

        tx.data[1] = len & 0xFF;

        memcpy(&tx.data[2], data, 6);

        tx.data_length = 8;

        CAN.write(tx);
    }

    uint16_t offset = 6;

    uint8_t seq = 1;

    while (offset < len)
    {
        CanMsg tx{};

        tx.id = _node_id;

        tx.data[0] =
            (ISO_TP_CONSECUTIVE_FRAME << 4) |
            (seq & 0x0F);

        uint16_t remaining = len - offset;

        uint8_t copy_len =
            (remaining >= 7) ? 7 : remaining;

        memcpy(
            &tx.data[1],
            &data[offset],
            copy_len
        );

        tx.data_length = copy_len + 1;

        CAN.write(tx);

        offset += copy_len;

        seq++;
    }
}

void Can::sendLegTelemetry()
{
    uint8_t payload[32];

    memset(payload, 0, sizeof(payload));

    payload[0] = CMD_LEG_STATE;

    //TODO - replace placeholder values with real axis state

    float positions[3] =
    {
        _leg->axes[0].getCurrentPos(),
        _leg->axes[1].getCurrentPos(),
        _leg->axes[2].getCurrentPos()
    };

    //TODO
    float torques[3] =
    {
        //TODO either make _estimates_torque public or add a getter in Axis class
        0, //_leg->axes[0].current_torque,
        0, //_leg->axes[1].current_torque,
        0, //_leg->axes[2].current_torque
    };

    memcpy(&payload[1],  positions, sizeof(positions));
    memcpy(&payload[13], torques, sizeof(torques));

    uint16_t payload_len =
        1 +
        sizeof(positions) +
        sizeof(torques);

    sendIsoTp(payload, payload_len);

}

void Can::handleCommandPayload(const uint8_t* d, uint16_t len)
{
    if (len == 0)
    {
        return;
    }

    uint8_t cmd = d[0];

    switch (cmd)
    {
        default:
        {
            Serial.println("CAN: Unsupported command received");
            Serial.println(cmd, HEX);

            return;
        }

        case CMD_LINEAR_MOVE:
        {
            float x, y, z, target_speed;
            memcpy(&x, &d[1], sizeof(float));
            memcpy(&y, &d[1+sizeof(float)], sizeof(float));
            memcpy(&z, &d[1+2*sizeof(float)], sizeof(float));
            memcpy(&target_speed, &d[1+3*sizeof(float)], sizeof(float));

            queueCommand([this, x, y, z, target_speed]()
            {
                _leg->linearMoveSetup(x, y, z, target_speed);
            });

            return;
        }

        case CMD_AUTO_TUNE:
        {
            Serial.println("CAN: Auto tune command received");

            queueCommand([this]()
            {
                //TODO - execute auto tune
            });

            return;
        }

        case CMD_QUADRATIC_MOVE:
        {
            Serial.println("CAN: Quadratic move command received");

            queueCommand([this]()
            {
                //TODO - execute quadratic move
            });

            return;
        }

        case CMD_SINGLE_AXIS_MOVE:
        {
            if (len < 6)
            {
                Serial.println("CAN: Invalid single axis move payload");
                return;
            }

            uint8_t axis = d[1];

            if (axis >= NUM_AXES_PER_LEG)
            {
                Serial.println("CAN: Invalid axis index");
                return;
            }

            float pos;
            memcpy(&pos, &d[2], sizeof(float));
            if (LOG_LEVEL >= BASIC_DEBUG)
            {
                Serial.printf(
                    "CAN: Single axis move | leg %d | axis %d | pos %.3f\n",
                    _leg_number,
                    axis,
                    pos
                );
            }

            queueCommand([this, axis, pos]()
            {
                _leg->axes[axis].setTargetPos(pos);
            });

            return;
        }

        case CMD_RAPID_MOVE:
        {
            float x, y, z;
            memcpy(&x, &d[1], sizeof(float));
            memcpy(&y, &d[1+sizeof(float)], sizeof(float));
            memcpy(&z, &d[1+2*sizeof(float)], sizeof(float));

            queueCommand([this, x, y, z]()
            {
                _leg->rapidMove(x, y, z);
            });

            return;
        }
    }
}

uint8_t Can::handleCanMessage(const CanMsg& msg)
{
    if (msg.id != _node_id)
    {
        return 0;
    }

    const uint8_t* d = msg.data;

    uint8_t pci = (d[0] >> 4) & 0x0F;

    uint32_t now = millis();

    switch (pci)
    {
        case ISO_TP_SINGLE_FRAME:
        {
            uint8_t payload_len = d[0] & 0x0F;

            if (payload_len > 7)
            {
                Serial.println("CAN: Invalid single frame length");

                return 0;
            }

            handleCommandPayload(&d[1], payload_len);

            return 1;
        }

        case ISO_TP_FIRST_FRAME:
        {
            resetIsoTp();

            _isotp_rx.active = true;

            _isotp_rx.last_update = now;

            _isotp_rx.expected_size =
                ((d[0] & 0x0F) << 8) |
                d[1];

            memcpy(_isotp_rx.data, &d[2], 6);

            _isotp_rx.current_size = 6;

            _isotp_rx.sequence_number = 1;

            Serial.printf(
                "CAN: ISO-TP first frame | expected %d bytes\n",
                _isotp_rx.expected_size
            );

            return 1;
        }

        case ISO_TP_CONSECUTIVE_FRAME:
        {
            if (!_isotp_rx.active)
            {
                Serial.println("CAN: Unexpected consecutive frame");

                return 1;
            }

            uint8_t seq = d[0] & 0x0F;

            if (seq != (_isotp_rx.sequence_number & 0x0F))
            {
                Serial.println("CAN: ISO-TP sequence mismatch");

                resetIsoTp();

                return 1;
            }

            _isotp_rx.last_update = now;

            uint16_t remaining =
                _isotp_rx.expected_size -
                _isotp_rx.current_size;

            uint8_t copy_len =
                (remaining >= 7) ? 7 : remaining;

            memcpy(
                &_isotp_rx.data[_isotp_rx.current_size],
                &d[1],
                copy_len
            );

            _isotp_rx.current_size += copy_len;

            _isotp_rx.sequence_number++;

            if (_isotp_rx.current_size >= _isotp_rx.expected_size)
            {
                Serial.printf(
                    "CAN: ISO-TP complete | %d bytes\n",
                    _isotp_rx.current_size
                );

                handleCommandPayload(
                    _isotp_rx.data,
                    _isotp_rx.expected_size
                );

                resetIsoTp();
            }

            return 1;
        }

        case ISO_TP_FLOW_CONTROL:
        {
            //TODO - transmit-side flow control support

            Serial.println("CAN: Flow control frame received");

            return 1;
        }

        default:
        {
            Serial.println("CAN: Unknown ISO-TP frame");

            return 1;
        }
    }
}

void Can::poll()
{
    uint32_t now = millis();

    if (_isotp_rx.active &&
        !isFresh(_isotp_rx.last_update))
    {
        Serial.println("CAN: ISO-TP timeout");

        resetIsoTp();
    }

    if ((now - _last_telemetry_tx) >= TELEMETRY_INTERVAL_MS)
    {
        _last_telemetry_tx = now;

        sendLegTelemetry();
    }
}