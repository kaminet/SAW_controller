#include <Arduino.h>

// ---- PREPARE I/O
#define LED 13           // onboard LED
#define myStartButton 16 // A2 PC2 Button Pin
#define migRelayPin 17   // A3 PC3 Relay for UVC LAMP

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
    // digitalWrite(LED, true);
    // delay(10000);
    // digitalWrite(LED, false);
    // delay(1000);
    digitalWrite(LED, digitalRead(myStartButton));
}