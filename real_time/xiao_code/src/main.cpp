#include <math.h>
#include <stdbool.h>
#include <Arduino.h>
#include <string.h>
#include "leg.hpp"
#include "can.hpp"
#include <RP2040_PWM.h>
#include <hardware/watchdog.h>
#include "hardware/resets.h"


Leg leg;
void handleCAN();

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Starting...");

  // for (uint8_t i = 0; i < 20; i++) {
    uint32_t reasons = watchdog_hw->reason;

    Serial.printf("watchdog reason: 0x%08lx\n", reasons);
    // delay(1000);
  // }
  leg.begin();

  leg.initializeAxes(LEG_NUMBER);
  leg.can->begin();
  leg.setAxisControlConstants(0, 20.0, 0.015, 3.0, 4.500, 0.0);
  leg.setAxisControlConstants(1, 20.0, 0.015, 3.0, 4.500, 0.0);
  leg.setAxisControlConstants(2, 20.0, 0.015, 6.0, 12.500, 0.0);
  // leg.setAxisTargetPos(0, 0.00);
  // leg.setAxisTargetPos(1, 0.00);
  // leg.setAxisTargetPos(2, 0.00);
  leg.rapidMove(0, 100.0, -240.0);
  
  if (watchdog_caused_reboot()) {
      Serial.println("Watchdog reboot");
      while (true) {
          // Blink LED to indicate watchdog reboot
          digitalWrite(LED_BUILTIN, HIGH);
          delay(500);
          digitalWrite(LED_BUILTIN, LOW);
          delay(500);
      }
  }
}

void loop() {
  static float dir = 1.0;
  handleCAN();
  if (leg.linearMovePerform() == 0) {
    // leg.linearMoveSetup(150.0 * dir, 112.0, -220.0, 200.0, false);
    // dir = -dir;
  }
  leg.processCommandQueue();
  leg.runSpeed();
}

void handleCAN()
{
    if (leg.can)
    {
        if (CAN.available())
        {
        
            while (CAN.available())
            {
                CanMsg msg = CAN.read();
                leg.can->handleCanMessage(msg);
            }
        }
        leg.can->poll();
    }
}
