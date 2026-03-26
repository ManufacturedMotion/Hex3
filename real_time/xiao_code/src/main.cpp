#include <math.h>
#include <stdbool.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>
#include "leg.hpp"
#include <RP2040_PWM.h>

#define INPUT_I2C_ADDRESS 13


Leg leg;

// I2C receive buffer and flag. The onReceive callback stores raw bytes here
// and sets i2cReceived = true. The main loop will copy and parse the buffer.
static volatile bool i2cReceived = false;
static volatile size_t i2cLen = 0;
static char i2cBuf[256];

void receiveI2C(int howMany) {
  size_t i = 0;
  // read as many bytes as available up to the buffer size - 1 (for null)
  while (Wire1.available() && i < (sizeof(i2cBuf) - 1)) {
    i2cBuf[i++] = (char)Wire1.read();
    Serial.println(i2cBuf[i-1]);
    Serial.flush();
  }
  i2cBuf[i] = '\0';
  // if extra bytes remain, consume them (avoid leaving them on the bus)
  while (Wire1.available()) Wire1.read();
  i2cLen = i;
  i2cReceived = true;
}
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Starting...");
  leg.begin();

  leg.initializeAxes(LEG_NUMBER);
  leg.setAxisControlConstants(0, 16.9, 20.0, 0.02, 3.0, 2.500, 1.0);
  leg.setAxisControlConstants(1, 16.9, 20.0, 0.02, 3.0, 2.500, 1.0);
  leg.setAxisControlConstants(2, 16.9, 20.0, 0.02, 3.0, 2.500, 1.0);
  leg.rapidMove(150.0, 150.0, -200.0);

  Wire1.begin(INPUT_I2C_ADDRESS);
  Wire1.onReceive(receiveI2C);
  Serial.print("I2C slave ready at address 0x"); Serial.println(INPUT_I2C_ADDRESS, HEX);
}

void loop() {
  static double speed = 0.0; //rads per second
  static bool stop = true;
  bool newCommand = false;
  String receivedCommand = "";
  if (i2cReceived) {
    noInterrupts();
    size_t len = i2cLen;
    i2cLen = 0;
    i2cReceived = false;
    char localBuf[sizeof(i2cBuf)];
    memcpy(localBuf, (const void*)i2cBuf, len + 1);
    interrupts();
    receivedCommand = String(localBuf);
    receivedCommand.trim();
    newCommand = true;
  }

  if (Serial.available()) {
    receivedCommand = Serial.readStringUntil('\n');
    newCommand = true;
  }


  if (newCommand) {
    Serial.print("Received command: ");
    Serial.println(receivedCommand);
    JsonDocument
     doc;
    DeserializationError err = deserializeJson(doc, receivedCommand.c_str());
    if (err) {
      Serial.print("JSON parse failed (I2C): ");
      Serial.println(err.c_str());
      Serial.print("Received command: ");
      Serial.println(receivedCommand);
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
          Serial.println("Setting positions directly (no PID control)");
          leg.setAxisTargetVelocity(0, x);
          leg.setAxisTargetVelocity(1, y);
          leg.setAxisTargetVelocity(2, z);
          break;
      }
    }
  }
  // static uint32_t last_move_time = 0;
  // if (millis() - last_move_time > 5000) {
  //   last_move_time = millis();
  //   leg.rapidMove(0.0, 112.5, -230.0);
  // } 
  leg.linearMovePerform();
  leg.runSpeed();
}


// #include <Wire.h>

// void receiveI2C() {
//   char buf[32];
//   size_t i = 0;
//   while (Wire1.available() && i < (sizeof(buf) - 1)) {
//     buf[i++] = (char)Wire1.read();
//   }
//   buf[i] = '\0';
//   Serial.print("Received I2C data: ");
//   Serial.println(buf);
//   Wire1.write("Hello from I2C slave!"); // Optionally send a response back
// }

// void receiveI2C1() {
//   char buf[32];
//   size_t i = 0;
//   while (Wire1.available() && i < (sizeof(buf) - 1)) {
//     buf[i++] = (char)Wire1.read();
//   }
//   buf[i] = '\0';
//   Serial.print("Received I2C data on Wire: ");
//   Serial.println(buf);
//   Wire.write("Hello from I2C slave!"); // Optionally send a response back
// }

// void receiveI2C(int howMany) {
//     receiveI2C();
// }

// void setup() {
//     Serial.begin(115200);
//     pinMode(INPUT_PULLUP, 6);
//     pinMode(INPUT_PULLUP, 7);
//     Wire1.begin(13);
//     Wire.begin(14);
//     Serial.println("I2C Scanner Ready");
//     Wire1.onRequest(receiveI2C);
//     Wire.onRequest(receiveI2C1);
//     Wire.onReceive(receiveI2C);
//     Wire1.onReceive(receiveI2C);
// }

// void loop() {
//     // Nothing to do in the main loop for this simple I2C slave example
//   delay(500);
// }