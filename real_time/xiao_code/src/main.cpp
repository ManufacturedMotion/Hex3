#include <math.h>
#include <stdbool.h>
#include <Arduino.h>
#include "leg.hpp"

Leg leg;


void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Starting...");
  leg.begin();
  leg.initializeAxes(0);
  //leg.axes[0].setSpeed(250);
  //leg.axes[0].moveToPos(3.14);
  //S1.moveToPosAtSpeed(3.14, 100);
}

void loop() {

  if (Serial.available() > 0){
    float receivedValue = Serial.parseFloat();  
    Serial.print("Received: ");
    Serial.println(receivedValue, 4);     
    //S1.moveToPos(receivedValue);
    leg.axes[0].moveToPosAtSpeed(receivedValue, 100);
  }

  double test = leg.axes[0].getCurrentPos();
  Serial.print("encoder says ");
  Serial.println(test);
  //leg.axes[0].runSpeed();
  delay(100);
}
