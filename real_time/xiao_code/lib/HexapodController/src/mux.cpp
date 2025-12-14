#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "mux.hpp"

Mux::Mux() {}

void Mux::begin(){
    if (_initialized) {
        return;
    }
    Wire.begin();
    while (!_initialized){
        Wire.beginTransmission(MUX_ADDR);
        if (Wire.endTransmission() == 0) {
            _initialized = true;
        }
        delay(2);   
    }
}

void Mux::setChannel(uint8_t channel) {
    if (channel > 7) {
        return;
    }
    if (channel  == _last_channel) {
        return;
    }
    Wire.beginTransmission(MUX_ADDR);
    /*
    Wire.write(0x00);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        //failed mux reset 
        return;
    }   
    delay(1); //TODO
    Wire.beginTransmission(MUX_ADDR);
    */
    Wire.write(1 << channel);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        //failed mux channel set
        return;
    }
    delay(1); //TODO nonblocking??
}

double Mux::readEncoder(uint8_t channel) {
    setChannel(channel);
    delayMicroseconds(2000);
    Wire.beginTransmission(ENC_ADDR);
    Wire.write(0x0E); 
    Wire.endTransmission();
    Wire.requestFrom(ENC_ADDR, 2);
    if (Wire.available() == 2) {
        uint8_t highByte = Wire.read();
        uint8_t lowByte  = Wire.read();
        uint16_t rawAngle = ((highByte & 0x0F) << 8) | lowByte; 
        double degrees = (rawAngle * 360.0) / 4096.0;
        return degrees;
    }
    return -1200;
}