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
    //leg.axes[2].setSpeed(100);    
    //leg.axes[2].moveToPos(receivedValue);
    //leg.axes[2].moveToPosAtSpeed(receivedValue, 100);
    for (int i = 0; i < 3; i++){
      leg.axes[i].moveToPosAtSpeed(receivedValue, 100);
    }
  }

  /*
  double test = leg.axes[2].getCurrentPos();
  Serial.print("encoder says ");
  Serial.println(test);
  leg.axes[2].runSpeed();
  delay(100);
  */
 leg.runSpeed();
 leg.toePressed();
 delay(100);

}