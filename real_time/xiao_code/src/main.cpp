#include <math.h>
#include <stdbool.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include "leg.hpp"
#include <RP2040_PWM.h>

Leg leg;
void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Starting...");
  leg.begin();
  leg.initializeAxes(0);
  leg.setAxisPIDConstants(0, 200.0, 20.0, 3.5);
  leg.setAxisPIDConstants(1, 200.0, 20.0, 2.5);
  leg.setAxisPIDConstants(2, 300.0, 30.0, 2.5);
  leg.setAxisTargetPos(0, 0.0);
  leg.setAxisTargetPos(1, 0.0);
  leg.setAxisTargetPos(2, 0.0);
}

void loop() {
  static double speed = 0.0; //rads per second
  static bool stop = true;
  if (Serial.available() > 0){
    String receivedCommand = Serial.readStringUntil('\n');
    receivedCommand.trim();

    // Parse JSON for X, Y and Z values using ArduinoJson
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, receivedCommand.c_str());
    if (err) {
      Serial.print("JSON parse failed: ");
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
      if (doc.containsKey("S")) z = doc["S"].as<double>();
      else if (doc.containsKey("s")) speed = doc["s"].as<double>();

      Serial.print("Parsed JSON -> X: "); Serial.print(x, 6);
      Serial.print("  Y: "); Serial.print(y, 6);
      Serial.print("  Z: "); Serial.println(z, 6);

      switch(move_type) {
        case 0: //linearmove
          leg.linearMoveSetup(x, y, z, speed, false);
          break;
        case 1: //rapid move
          leg.rapidMove(x, y, z);
          break;
        case 2: //set positions directly
          leg.setAxisTargetPos(0, x);
          leg.setAxisTargetPos(1, y);
          leg.setAxisTargetPos(2, z);
          break;
      }
    }
  }
  // static double target_pos = 0.00; //some value to enter the loop
  // static bool going_up = true;
  // static uint32_t last_update_time = 0;
  // uint32_t delta = millis() - last_update_time;
  // if (!stop) {
  //   if (delta >= 10) {
  //     last_update_time = millis(); 
  //       if (target_pos >= 1.0) {
  //         going_up = false;
  //       }
  //       else if (target_pos <= -1.0) {
  //         going_up = true;
  //       }
  //       double increment = speed * (delta / 1000.0);
  //       target_pos += going_up ? increment : -increment;
  //       leg.setAxisTargetPos(2, target_pos);
  //   }
  // }
  leg.linearMovePerform();
  leg.runSpeed();
}