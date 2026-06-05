#include <stdbool.h>
#include <Arduino.h>
#include <stdint.h>

#ifndef FIFO_COMMAND
#define FIFO_COMMAND

#define COMMAND_QUEUE_SIZE 16

enum class CommandType : uint8_t
{
    SingleAxisMove,
    LinearMove,
    QuadraticMove,
    RapidMove,
    AutoTune
};

struct Command
{
    CommandType type;
    union //TODO add other commands as needed
    {
        struct
        {
            uint8_t axis;
            float position;
        } single_axis;

        struct
        {
            float positions[3];
            uint16_t move_time_ms;
        } linear_move;
    };
};

//using a ring buffer for command queue, with head/tail indices and count
class CommandQueue
{
    public:
        bool enqueue(const Command& cmd);
        bool dequeue(Command& cmd);
        bool isEmpty() const;
        uint8_t size() const;

    private:
        Command buffer[COMMAND_QUEUE_SIZE];
        volatile uint8_t head = 0;
        volatile uint8_t tail = 0;
        volatile uint8_t count = 0;
};

#endif