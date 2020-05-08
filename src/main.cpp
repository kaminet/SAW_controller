#include <Arduino.h>
#include "OneButton.h"

// ---- PREPARE I/O
#define LED 13           // onboard LED
#define myStartButton 16 // A2 PC2 Button Pin
#define migRelayPin 17   // A3 PC3 Relay for UVC LAMP

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

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);         // sets the digital pin as output
  pinMode(migRelayPin, OUTPUT); // sets the digital pin as output
}

void loop()
{
  // put your main code here, to run repeatedly:
  unsigned long now = millis();

  switch (nextAction)
  {
  case ACTION_OPEN:
    break;
  case ACTION_IDLE:
    break;
  case ACTION_DISINFECTION:
    break;
  case ACTION_FORCE_WAIT:
    break;
  case ACTION_FORCE_ON:
    break;
  default:
    // printf("please select correct initial state");  // This should never occur
    break;
  }
}