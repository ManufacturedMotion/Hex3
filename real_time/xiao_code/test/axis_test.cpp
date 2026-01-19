#include "axis.hpp"
#include <math.h>
#include <stdbool.h>
#include <Arduino.h>
#include "mux.hpp"

Mux mux;
Axis S1;

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Starting...");
  mux.begin();
  S1.link(D11, D12, 5, mux);
  S1.initializePositionLimits(0, 2 * M_PI);
  S1.setMapping(0, 1, false);
  S1.setSpeed(250);
  S1.moveToPos(3.14);
  //S1.moveToPosAtSpeed(3.14, 100);
}

void loop() {

  if (Serial.available() > 0){
    float receivedValue = Serial.parseFloat();  
    Serial.print("Received: ");
    Serial.println(receivedValue, 4);     
    //S1.moveToPos(receivedValue);
    S1.moveToPosAtSpeed(receivedValue, 100);
  }

  S1.runSpeed();
}
