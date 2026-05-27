#include <math.h>
#include <stdbool.h>
#include <Arduino.h>
#include <string.h>
#include "leg.hpp"
#include "can.hpp"
#include <RP2040_PWM.h>

Leg leg;
void handleCAN();

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Starting...");
  leg.begin();
  delay(2000);

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
  
  //handleCAN();
  if (leg.linearMovePerform() == 0) {
    // leg.linearMoveSetup(150.0 * dir, 112.0, -220.0, 200.0, false);
    // dir = -dir;
  }
  
  leg.runSpeed();
}

void handleCAN()
{
    uint32_t start = micros();
    const uint32_t BUDGET_US = 0; // adjust 100–500us

    if (CAN.available())
    {
        CanMsg msg = CAN.read();

        //TODO remove debug long term?
        #if LOG_LEVEL >= BASIC_DEBUG
        Serial.printf("CAN RX | ID: 0x%X | DATA: ", msg.id);
        for (int i = 0; i < msg.data_length; i++)
        {
            Serial.printf("%02X ", msg.data[i]);
        }
        Serial.println();
        #endif

        if (leg.can)
        {
            leg.can->handleCanMessage(msg);
        }

      if (micros() - start > BUDGET_US)
        return;
    }

    if (leg.can)
    {
        leg.can->poll();
        auto fn = leg.can->popPendingCommand();
        if (fn)
        {
            fn();
        }
    }
  }