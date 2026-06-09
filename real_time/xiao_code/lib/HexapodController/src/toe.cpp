#include "toe.hpp"
#include <Arduino.h>

Toe::Toe() {
    pinMode(TOE_PIN, INPUT); 
}

float Toe::read() {
    return analogRead(TOE_PIN);
}