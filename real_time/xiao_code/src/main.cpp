#include <math.h>
#include <stdbool.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <string.h>
#include "leg.hpp"
#include <RP2040_PWM.h>

#define INPUT_I2C_ADDRESS 0x10


Leg leg;

// I2C receive buffer and flag. The onReceive callback stores raw bytes here
// and sets i2cReceived = true. The main loop will copy and parse the buffer.
static volatile bool i2cReceived = false;
static volatile size_t i2cLen = 0;
static char i2cBuf[256];

void receiveI2C(int howMany) {
  size_t i = 0;
  // read as many bytes as available up to the buffer size - 1 (for null)
  while (Wire.available() && i < (sizeof(i2cBuf) - 1)) {
    i2cBuf[i++] = (char)Wire.read();
  }
  i2cBuf[i] = '\0';
  // if extra bytes remain, consume them (avoid leaving them on the bus)
  while (Wire.available()) Wire.read();
  i2cLen = i;
  i2cReceived = true;
}
void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Starting...");
  leg.begin();

  leg.initializeAxes(LEG_NUMBER);
  leg.setAxisControlConstants(0, 300.0, 30.0, 2.5, 1.0, 0.05);
  leg.setAxisControlConstants(1, 300.0, 20.0, 2.5, 1.0, 0.05);
  leg.setAxisControlConstants(2, 300.0, 30.0, 2.5, 1.0, 0.05);
  leg.setAxisTargetPos(0, 0.0);
  leg.setAxisTargetPos(1, 0.0);
  leg.setAxisTargetPos(2, 0.0);

  Wire.begin(INPUT_I2C_ADDRESS);
  Wire.onReceive(receiveI2C);
  Serial.print("I2C slave ready at address 0x"); Serial.println(INPUT_I2C_ADDRESS, HEX);
}

void loop() {
  static double speed = 0.0; //rads per second
  static bool stop = true;
  if (i2cReceived) {
    noInterrupts();
    size_t len = i2cLen;
    i2cLen = 0;
    i2cReceived = false;
    char localBuf[sizeof(i2cBuf)];
    memcpy(localBuf, (const void*)i2cBuf, len + 1);
    interrupts();

    String receivedCommand = String(localBuf);
    receivedCommand.trim();

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, receivedCommand.c_str());
    if (err) {
      Serial.print("JSON parse failed (I2C): ");
      Serial.println(err.c_str());
    } else {
      double x = 0.0, y = 0.0, z = 0.0, speed = 0.0;
      uint8_t move_type = 0;
      if (doc.containsKey("T")) move_type = doc["T"].as<int>();
      else if (doc.containsKey("t")) move_type = doc["t"].as<int>();
      if (doc.containsKey("X")) x = doc["X"].as<double>();
      else if (doc.containsKey("x")) x = doc["x"].as<double>();
      if (doc.containsKey("Y")) y = doc["Y"].as<double>();
      else if (doc.containsKey("y")) y = doc["y"].as<double>();
      if (doc.containsKey("Z")) z = doc["Z"].as<double>();
      else if (doc.containsKey("z")) z = doc["z"].as<double>();
      if (doc.containsKey("S")) speed = doc["S"].as<double>();
      else if (doc.containsKey("s")) speed = doc["s"].as<double>();

      Serial.print("I2C Parsed JSON -> X: "); Serial.print(x, 6);
      Serial.print("  Y: "); Serial.print(y, 6);
      Serial.print("  Z: "); Serial.println(z, 6);

      switch(move_type) {
        case 0: // linearmove
          leg.linearMoveSetup(x, y, z, speed, false);
          break;
        case 1: // rapid move
          leg.rapidMove(x, y, z);
          break;
        case 2: // set positions directly
          leg.setAxisTargetPos(0, x);
          leg.setAxisTargetPos(1, y);
          leg.setAxisTargetPos(2, z);
          break;
      }
    }
  }
  leg.linearMovePerform();
  leg.runSpeed();
}