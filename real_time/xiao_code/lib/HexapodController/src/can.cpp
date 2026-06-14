#include "can.hpp"
#include "leg.hpp"
#include "log_levels.hpp"
#include <functional>

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
Byte 0      CMD_LINEAR_MOVE
Byte 1..2   int16 x (scaled by 10)
Byte 3..4   int16 y (scaled by 10)
Byte 5..6   int16 z (scaled by 10)
Byte 7..8   int16 speed (scaled by 10)

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
Byte 2..3   -> int16 position (scaled by 10)

Single frame

--------------------------------------------------
CMD_RAPID_MOVE (0x14)
--------------------------------------------------
Rapid move for one leg

Payload:
Byte 0      -> command id
Byte 1..2   int16 x (scaled by 10)
Byte 3..4   int16 y (scaled by 10)
Byte 5..6   int16 z (scaled by 10)

Can be single or multi-frame

--------------------------------------------------
CMD_LEG_STATE (0x20)
--------------------------------------------------
Periodic leg telemetry message

Payload:
Byte 0       -> command id
Byte 1..6    -> 3 int16 positions (scaled by 10)
Byte 7..8   -> 3 int16 toe compression (scaled by 10)

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
      _tx_node_id(0x180 + leg_number),
      _rx_node_id(0x100 + leg_number),
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
        Serial.printf("CAN tx/rx ready: 0x%X, 0x%X\n", _tx_node_id, _rx_node_id);
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

static float decodeScaledInt16(int16_t raw)
{
    return static_cast<float>(raw) / 10.0f;
}

static int16_t encodeScaledInt16(float value)
{
    return static_cast<int16_t>(value * 10.0f + (value >= 0.0f ? 0.5f : -0.5f));
}

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

    #if LOG_LEVEL >= CAN_DEBUG
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

        tx.id = _tx_node_id;

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

        tx.id = _tx_node_id;

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

        tx.id = _tx_node_id;

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

    int16_t positions[3] =
    {
        encodeScaledInt16(_leg->axes[0].getCurrentPos()),
        encodeScaledInt16(_leg->axes[1].getCurrentPos()),
        encodeScaledInt16(_leg->axes[2].getCurrentPos())
    };
    int16_t toe_compression = encodeScaledInt16(_leg->readToe());
    memcpy(&payload[1], positions, sizeof(positions));
    memcpy(&payload[7], &toe_compression, sizeof(toe_compression));

    uint16_t payload_len =
        1 +
        sizeof(positions) +
        sizeof(toe_compression);

    sendIsoTp(payload, payload_len);

    //#if LOG_LEVEL >= CAN_DEBUG
    //   Serial.println("CAN: Leg telemetry sent");
    //#endif
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
            #if LOG_LEVEL >= CAN_DEBUG
                Serial.println("CAN: Unsupported command received");
                Serial.println(cmd, HEX);
            #endif
            return;
        }

        case CMD_LINEAR_MOVE:
        {
            if (len < 10)
            {
                #if LOG_LEVEL >= CAN_DEBUG
                    Serial.println("CAN: Invalid linear move payload");
                #endif
                return;
            }

            int16_t x_raw = 0;
            int16_t y_raw = 0;
            int16_t z_raw = 0;
            int16_t speed_raw = 0;

            memcpy(&x_raw,     &d[1],  sizeof(int16_t));
            memcpy(&y_raw,     &d[3],  sizeof(int16_t));
            memcpy(&z_raw,     &d[5],  sizeof(int16_t));
            memcpy(&speed_raw, &d[7],  sizeof(int16_t));

            Command command{};
            command.type = CommandType::LinearMove;
            command.linear_move.x = decodeScaledInt16(x_raw);
            command.linear_move.y = decodeScaledInt16(y_raw);
            command.linear_move.z = decodeScaledInt16(z_raw);
            command.linear_move.speed = decodeScaledInt16(speed_raw);

            if (LOG_LEVEL >= CAN_DEBUG)
            {
                Serial.printf(
                    "CAN: Linear move | leg %d | "
                    "x %.3f y %.3f z %.3f speed %.3f rel %d\n",
                    _leg_number,
                    command.linear_move.x,
                    command.linear_move.y,
                    command.linear_move.z,
                    command.linear_move.speed
                );
            }
            _leg->command_queue.enqueue(command);
            return;
        }

        case CMD_AUTO_TUNE:
        {
            #if LOG_LEVEL >= CAN_DEBUG
                Serial.println("CAN: Auto tune command received");
            #endif
            //TODO
            return;
        }

        case CMD_QUADRATIC_MOVE:
        {
            #if LOG_LEVEL >= CAN_DEBUG
                Serial.println("CAN: Quadratic move command received");
            #endif
            //TODO
            return;
        }

        case CMD_SINGLE_AXIS_MOVE:
        {
            if (len < 4)
            {
                #if LOG_LEVEL >= CAN_DEBUG
                    Serial.println("CAN: Invalid single axis move payload");
                #endif
                return;
            }

            uint8_t axis = d[1];
            if (axis >= NUM_AXES_PER_LEG)
            {
                #if LOG_LEVEL >= CAN_DEBUG
                    Serial.println("CAN: Invalid axis index");
                #endif
                return;
            }

            int16_t raw_pos = 0;
            memcpy(&raw_pos, &d[2], sizeof(int16_t));
            float pos = decodeScaledInt16(raw_pos);
            if (LOG_LEVEL >= CAN_DEBUG)
            {
                Serial.printf(
                    "CAN: Single axis move | leg %d | axis %d | pos %.3f\n",
                    _leg_number,
                    axis,
                    pos
                );
            }

            Command command{};
            command.type = CommandType::SingleAxisMove;
            command.single_axis.axis = axis;
            command.single_axis.position = pos;
            _leg->command_queue.enqueue(command);
            return;
        }

        case CMD_RAPID_MOVE:
        {
            if (len < 7)
            {
                #if LOG_LEVEL >= CAN_DEBUG
                    Serial.println("CAN: Invalid rapid move payload");
                #endif
                return;
            }

            int16_t x_raw = 0;
            int16_t y_raw = 0;
            int16_t z_raw = 0;

            memcpy(&x_raw, &d[1], sizeof(int16_t));
            memcpy(&y_raw, &d[3], sizeof(int16_t));
            memcpy(&z_raw, &d[5], sizeof(int16_t));

            float x = decodeScaledInt16(x_raw);
            float y = decodeScaledInt16(y_raw);
            float z = decodeScaledInt16(z_raw);

            if (LOG_LEVEL >= CAN_DEBUG)
            {
                Serial.printf(
                    "CAN: Rapid move | leg %d | x %.3f | y %.3f | z %.3f\n",
                    _leg_number,
                    x,
                    y,
                    z
                );
            }

            Command command{};
            command.type = CommandType::RapidMove;
            command.rapid_move.x = x;
            command.rapid_move.y = y;
            command.rapid_move.z = z;

            _leg->command_queue.enqueue(command);

            return;
        }
    }
}
            
void Can::handleCanMessage(const CanMsg& msg)
{
    if (msg.id != _rx_node_id)
    {
        return;
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
                #if LOG_LEVEL >= CAN_DEBUG
                    Serial.println("CAN: Invalid single frame length");
                #endif
                return;
            }

            handleCommandPayload(&d[1], payload_len);

            return;
        }

        case ISO_TP_FIRST_FRAME:
        {
            resetIsoTp();

            _isotp_rx.active = true;
            _isotp_rx.last_update = now;

            _isotp_rx.expected_size =
                ((d[0] & 0x0F) << 8) |
                d[1];

            if (_isotp_rx.expected_size > sizeof(_isotp_rx.data))
            {
                CanMsg fc{};

                fc.id = _tx_node_id;

                fc.data[0] =
                    (ISO_TP_FLOW_CONTROL << 4) |
                    0x02; // OVFLW

                fc.data[1] = 0;
                fc.data[2] = 0;

                fc.data_length = 3;

                CAN.write(fc);

                resetIsoTp();
                return;
            }

            memcpy(_isotp_rx.data, &d[2], 6);
            _isotp_rx.current_size = 6;
            _isotp_rx.sequence_number = 1;
            CanMsg fc{};
            fc.id = _tx_node_id;

            fc.data[0] =
                (ISO_TP_FLOW_CONTROL << 4) |
                0x00; // CTS

            fc.data[1] = 0; // block size = unlimited
            fc.data[2] = 0; // STmin = 0 ms

            fc.data_length = 3;

            CAN.write(fc);

            return;
        }

        case ISO_TP_CONSECUTIVE_FRAME:
        {
            if (!_isotp_rx.active)
            {
                #if LOG_LEVEL >= CAN_DEBUG
                    Serial.println("CAN: Unexpected consecutive frame");
                #endif
                return;
            }

            uint8_t seq = d[0] & 0x0F;

            if (seq != (_isotp_rx.sequence_number & 0x0F))
            {
                #if LOG_LEVEL >= CAN_DEBUG
                    Serial.println("CAN: ISO-TP sequence mismatch");
                #endif
                resetIsoTp();
                return;
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
                #if LOG_LEVEL >= CAN_DEBUG
                    Serial.printf(
                        "CAN: ISO-TP complete | %d bytes\n",
                        _isotp_rx.current_size
                    );
                #endif

                handleCommandPayload(
                    _isotp_rx.data,
                    _isotp_rx.expected_size
                );
                resetIsoTp();
            }

            return;
        }

        case ISO_TP_FLOW_CONTROL:
        {
            //TODO - transmit-side flow control support
            //#if LOG_LEVEL >= CAN_DEBUG
            //    Serial.println("CAN: Flow control frame received");
            //#endif
            return;
        }

        default:
        {
            #if LOG_LEVEL >= CAN_DEBUG
                Serial.println("CAN: Unknown ISO-TP frame");
            #endif
            return;
        }
    }
}

void Can::poll()
{
    uint32_t now = millis();

    if (_isotp_rx.active &&
        !isFresh(_isotp_rx.last_update))
    {
        #if LOG_LEVEL >= CAN_DEBUG
            Serial.println("CAN: ISO-TP timeout");
        #endif

        resetIsoTp();
    }

    if ((now - _last_telemetry_tx) >= TELEMETRY_INTERVAL_MS)
    {
        _last_telemetry_tx = now;

        sendLegTelemetry();
    }
}
