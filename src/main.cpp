#include <Arduino.h>
#include "OneButton.h"

// ---- PREPARE I/O
#define myLED 13           // onboard LED
#define myStartButton 16 // A2 PC2 Button Pin
#define myRelayPin 17   // A3 PC3 Relay for UVC LAMP
#define uvcTime 600000UL   // disinfection time 1m
#define idleTime 1800000UL   // idle time 30m
#define doorTime 2000UL   // door seafty time

unsigned long curTime = 0UL;   // will store current time to avoid multiple millis() calls
unsigned long actionDutation = 0UL;   // will store time when action should end
bool firstLong = false;  // first UvcTime cycle longer

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

// Setup a new OneButton on pin myStartButton.  
OneButton button(myStartButton, true);

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
    // This should never occur
    break;
  }
}

// this function will be called when the button was pressed 1 time and them some time has passed.
void myClickFunction() {
  switch (nextAction)
  {
    case ACTION_FORCE_WAIT:
      nextAction = ACTION_FORCE_ON;
      break;
    case ACTION_FORCE_ON:
      nextAction = ACTION_FORCE_WAIT;
      break;
    default:

    break;
  }
} // myClickFunction

// this function will be called when the button was pressed 2 times in a short timeframe.
void myDoubleClickFunction() {
  switch (nextAction)
  {
    case ACTION_FORCE_WAIT:
      nextAction = ACTION_OPEN;
      break;
    case ACTION_FORCE_ON:
      nextAction = ACTION_OPEN;
      break;
    case ACTION_OPEN:
      if (firstLong == true)
      {
         firstLong = false;
         nextAction = ACTION_FORCE_WAIT;
      } else {
        firstLong = true;
      }
      break;
    default:
    // This should never occur
    break;
  }
} // myDoubleClickFunction

// this function will be called when the button was pressed for one second.
void myLongPressFunction() {
  switch (nextAction)
  {
    case ACTION_OPEN:
      nextAction = ACTION_DISINFECTION;
      statusLed = LED_FAST;
      if (firstLong == true)
      { actionDutation = curTime + uvcTime * 4;
         firstLong = false;
      } else {
        actionDutation = curTime + uvcTime;
      }
      break;
    default:
    
    break;
  }
} // myLongPressFunction

void setup()
{
  // put your setup code here, to run once:
  pinMode(myLED, OUTPUT);         // sets the digital pin as output
  pinMode(myRelayPin, OUTPUT); // sets the digital pin as output
  // link the myClickFunction function to be called on a click event.   
  button.attachClick(myClickFunction);

  // link the doubleclick function to be called on a doubleclick event.   
  button.attachDoubleClick(myDoubleClickFunction);

  // link the longpress function to be called on a longpress event.   
  button.attachLongPressStart(myLongPressFunction);

  // set 80 msec. debouncing time. Default is 50 msec.
  button.setDebounceTicks(80);
} // setup

void loop()
{
  // put your main code here, to run repeatedly:
  curTime = millis();
  statusLED();
  button.tick(); // keep watching the push button:

  switch (nextAction)
  {
  case ACTION_OPEN:
    statusLed = LED_SLOW;
    digitalWrite(myRelayPin, LOW);
    break;
  case ACTION_IDLE:
    if (curTime > actionDutation)
    {
      nextAction = ACTION_DISINFECTION;
      actionDutation = curTime + uvcTime;
    }
    if (digitalRead(myStartButton) == false)
    {nextAction = ACTION_OPEN;}
    break;
  case ACTION_DISINFECTION:
    if (curTime > actionDutation)
    {
      nextAction = ACTION_IDLE;
      actionDutation = curTime + idleTime;
    }
    if (digitalRead(myStartButton) == false)
    {nextAction = ACTION_OPEN;
    digitalWrite(myRelayPin, LOW);
    } else {
      digitalWrite(myRelayPin, HIGH);
    }
    break;
  case ACTION_FORCE_WAIT:
    statusLed = LED_ON;
    digitalWrite(myRelayPin, LOW);
    break;
  case ACTION_FORCE_ON:
    statusLed = LED_FAST;
    digitalWrite(myRelayPin, HIGH);
    break;
  default:
    // printf("please select correct initial state");  // This should never occur
    break;
  }
}
