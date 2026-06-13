#include <math.h>
#include <stdbool.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>
#include "leg.hpp"
#include <RP2040_PWM.h>
#include "can.hpp"

#define LOG_LEVEL 1
Can can(1, 0, CanBitRate::BR_500k, 3);  

void setup() {
  Serial.begin(115200);
  delay(9000);
  can.begin();
}

void loop() {

  

    while (CAN.available())
    {
        CanMsg msg = CAN.read();            
        
        Serial.printf("CAN RX | ID: 0x%X | DATA: ", msg.id);

        for (int i = 0; i < msg.data_length; i++)
        {
            Serial.printf("%02X ", msg.data[i]);
        }

        Serial.println();

        can.handleCanMessage(msg);  
    }

    can.poll(); // Handle any pending actions based on received CAN messages

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