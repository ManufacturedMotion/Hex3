#include <stdbool.h>
#include <Arduino.h>
#include "command_queue.hpp"
#include "config.hpp"

bool CommandQueue::enqueue(const Command& cmd)
{
    if (count >= COMMAND_QUEUE_SIZE)
    {
        // queue full
        Serial.println("Command queue FULL - dropping command");
        return false;
    }

    buffer[tail] = cmd;
    tail = (tail + 1) % COMMAND_QUEUE_SIZE;
    count++;

    return true;
}

bool CommandQueue::dequeue(Command& cmd)
{
    if (count == 0)
    {
        return false;
    }

    cmd = buffer[head];
    head = (head + 1) % COMMAND_QUEUE_SIZE;
    count--;

    return true;
}

bool CommandQueue::isEmpty() const
{
    return count == 0;
}

uint8_t CommandQueue::size() const
{
    return count;
}
