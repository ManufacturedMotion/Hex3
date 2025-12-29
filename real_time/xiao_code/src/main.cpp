#include <math.h>
#include <stdbool.h>
#include <Arduino.h>
#include "leg.hpp"
#include <RP2040_PWM.h>

Leg leg;
void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Starting...");
  leg.begin();
  leg.initializeAxes(0);
  leg.setAxisPIDConstants(1, 0.5, 0.0, 0.0);
}

void loop() {
  if (Serial.available() > 0){
    String receivedCommand = Serial.readStringUntil('\n');
    float receivedValue = receivedCommand.toFloat();
    Serial.print("Setting target pos to: ");
    Serial.println(receivedValue);
    if (receivedValue > 0) {
      leg.setAxisTargetPos(1, receivedValue);
    }
    else {
      leg.stopAxis(1);
    }
  }
  leg.runSpeed();
}