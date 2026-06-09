#include <stdbool.h>
#include <Arduino.h>
#include <stdint.h>

#ifndef FIFO_COMMAND
#define FIFO_COMMAND

#define COMMAND_QUEUE_SIZE 1

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
            float x;
            float y;
            float z;
        } rapid_move;

        struct
        {
            float x;
            float y;
            float z;
            float speed;
            bool relative;
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