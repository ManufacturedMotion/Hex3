#include <math.h>
#include <stdbool.h>
#include <Arduino.h>
#include <string.h>
#include "leg.hpp"
#include <RP2040_PWM.h>



Leg leg;

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Starting...");
  leg.begin();

  leg.initializeAxes(LEG_NUMBER);
  leg.setAxisControlConstants(0, 20.0, 0.015, 3.0, 4.500, 0.0);
  leg.setAxisControlConstants(1, 20.0, 0.015, 3.0, 4.500, 0.0);
  leg.setAxisControlConstants(2, 20.0, 0.015, 3.0, 4.500, 0.0);
  // leg.setAxisTargetPos(0, 0.00);
  // leg.setAxisTargetPos(1, 0.00);
  // leg.setAxisTargetPos(2, 0.00);
  leg.rapidMove(0, 100.0, -240.0);
}

void loop() {

  static float dir = 1.0;

  static double speed = 0.0; //rads per second
  static bool stop = true;
  
  if (leg.linearMovePerform() == 0) {
    // leg.linearMoveSetup(150.0 * dir, 112.0, -220.0, 200.0, false);
    // dir = -dir;
  }
  
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