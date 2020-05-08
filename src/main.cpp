#include <Arduino.h>
#include "OneButton.h"

// ---- PREPARE I/O
#define myLED 13           // onboard LED
#define myStartButton 16 // A2 PC2 Button Pin
#define myRelayPin 17   // A3 PC3 Relay for UVC LAMP

unsigned long curTime = 0UL;   // will store current time to avoid multiple millis() calls

// The actions I ca do...
typedef enum
{
  ACTION_OPEN,         // set UVC "OFF"
  ACTION_IDLE,         // seafty delay timer
  ACTION_DISINFECTION, // set UVC "ON" timer
  ACTION_FORCE_WAIT,   // set UVC "OFF"
  ACTION_FORCE_ON      // set UVC "ON"
} MyActions;

MyActions nextAction = ACTION_OPEN; // no action when starting

// The actions I ca do...
typedef enum {
  LED_OFF,  // set LED "OFF".
  LED_ON,   // set LED "ON"
  LED_SLOW, // blink LED "SLOW"
  LED_FAST  // blink LED "FAST"
} 
LedState;

LedState statusLed = LED_OFF; // Led disabled on startup

void statusLED() {
  switch (statusLed)
  {
  case LED_OFF:
    // Turn LED off
    digitalWrite(myLED, LOW);
    break;
  case LED_ON:
    // turn LED on
    digitalWrite(myLED, HIGH);
    break;
  case LED_SLOW:
    // do a slow blinking
    if (curTime % 1000 < 500) {
      digitalWrite(myLED, LOW);
    } else {
      digitalWrite(myLED, HIGH);
    } // if
    break;
  case LED_FAST:
    // do a fast blinking
    if (curTime % 200 < 100) {
      digitalWrite(myLED, LOW);
    } else {
      digitalWrite(myLED, HIGH);
    } // if
    break;
  default:
    // printf("please select correct initial state");  // This should never occur
    break;
  }
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(myLED, OUTPUT);         // sets the digital pin as output
  pinMode(myRelayPin, OUTPUT); // sets the digital pin as output
}

void loop()
{
  // put your main code here, to run repeatedly:
  curTime = millis();
  statusLED();

  switch (nextAction)
  {
  case ACTION_OPEN:
    digitalWrite(myRelayPin, LOW);
    break;
  case ACTION_IDLE:
    break;
  case ACTION_DISINFECTION:
    digitalWrite(myRelayPin, HIGH);
    break;
  case ACTION_FORCE_WAIT:
    digitalWrite(myRelayPin, LOW);
    break;
  case ACTION_FORCE_ON:
    digitalWrite(myRelayPin, HIGH);
    break;
  default:
    // printf("please select correct initial state");  // This should never occur
    break;
  }
}