/*
 UVC-Timer.ino
 
 This is a sample sketch to operate chamber with Ultra Violet C... light for disinfection purposes. 
 
 Setup circuit:
 * Connect a pushbutton or hal sensor to pin A2 (motorPwm) and VCC, add 4,7 - 10 k resistor to GND for pulldown.
 * Connect relay control pin to pin A1 and UVC lamp to AC thru relay COM and NO terminals.
 * The pin 13 (ledSys) is used for output status of Finite State Machine by blinking built-in LED.
  
 When doors are closed sketch is cycling between UVC ON and OFF (1 min ON and 30 min OFF),
 if doors opens it goes to state OPENED.
 In OPENED state we can force first DISINFECTION cycle to 4 x normal time by doubleclick pushbutton.
 If we doubleclick pushbutton second time we go to FORCED_WAIT state
 We can switch between FORCED_WAIT and FORCED_ON states by single click or return to OPENED state by doubleclick.

 State-Diagram

      start
       |    +-----------d-click-----------+------------------------\
       V    V                             |                        |
     -----------                     --------------             ---|--------
  +>|  OPENED   |<--2 x d-click---->|  FORCE_WAIT  |<--click-->|  FORCE_ON  |
  |  -----------                     --------------             ------------
  button      | longpress
  release     V
  |  -----------
  +-| DISINFECT |
  |  -----------
  |      ^
  |      | timers
  |      V
  |  -----------
  +-|   IDLE    |
     -----------
 */

#include <Arduino.h>
#include "OneButton.h"

// #define FSM_DEBUG
#undef FSM_DEBUG

// ---- PREPARE I/O
#define ledSys 13             // PB5 onboard LED
#define ledUp 12              // PB4 LED for UP movement
#define ledDowm 11            // PB3 LED for DOWN movement
#define buttonUp 6            // PD6 Command UP Button Pin
#define buttonDown 5          // PD5 Command DOWN Button Pin
#define buttonEndstopUp 4     // PD4 UP limit Button Pin
#define buttonEndstopDown 2   // PD2 DOWN limitButton Pin
#define buttonEstop 3         // PD3 emergeny stop Button Pin
#define motorPwm 9            // PB1 Control motor speed
#define motorCw 8             // PB0 Command motor directopn CW
#define motorCcw 7            // PB7 Command motor directopn CCW
#if defined (FSM_DEBUG)
  #define uvcTime 3000UL   // disinfection time 1m
  #define idleTime 6000UL   // idle time 30m
#else
  #define uvcTime 1200000UL   // disinfection time 20m
  #define idleTime 3600000UL   // idle time 60m
  #define doorTime 2000UL   // door seafty time
#endif
unsigned long curTime = 0UL;   // will store current time to avoid multiple millis() calls
unsigned long actionDutation = 0UL;   // will store time when action should end
bool firstLong = false;  // first UvcTime cycle longer
bool pri1 = false;

// The actions I ca do...
typedef enum
{
  ACTION_OPEN,         // set UVC "OFF"
  ACTION_DISINFECTION, // set UVC "ON" with timer
  ACTION_IDLE,         // set UVC "OFF" with idle timer
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

// Setup a new OneButton on pin buttonUp.  
OneButton button(buttonUp, HIGH, false);

void statusLED() {
  switch (statusLed)
  {
  case LED_OFF:
    // Turn LED off
    digitalWrite(ledSys, LOW);
    break;
  case LED_ON:
    // turn LED on
    digitalWrite(ledSys, HIGH);
    break;
  case LED_SLOW:
    // do a slow blinking
    if (curTime % 1000 < 500) {
      digitalWrite(ledSys, LOW);
    } else {
      digitalWrite(ledSys, HIGH);
    } // if
    break;
  case LED_FAST:
    // do a fast blinking
    if (curTime % 200 < 100) {
      digitalWrite(ledSys, LOW);
    } else {
      digitalWrite(ledSys, HIGH);
    } // if
    break;
  default:
    // This should never occur
    break;
  }
}

// this function will be called when the button was pressed 1 time and them some time has passed.
void myClickFunction() {
#if defined (FSM_DEBUG)
  Serial.println(F("BUTTON!"));
#endif
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
#if defined (FSM_DEBUG)
  Serial.println(F("BUTTON! DOUBLE"));
#endif
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
#if defined (FSM_DEBUG)
  Serial.println(F("BUTTON! LONG"));
#endif
  switch (nextAction)
  {
    case ACTION_OPEN:
      nextAction = ACTION_DISINFECTION;
      if (firstLong == true)
      { actionDutation = curTime + uvcTime * 2;
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
  pinMode(ledSys, OUTPUT);         // sets the digital pin as output
  pinMode(motorPwm, OUTPUT); // sets the digital pin as output
  
  // link the myClickFunction function to be called on a click event.   
  button.attachClick(myClickFunction);

  // link the doubleclick function to be called on a doubleclick event.   
  button.attachDoubleClick(myDoubleClickFunction);

  // link the longpress function to be called on a longpress event.   
  button.attachLongPressStart(myLongPressFunction);

  // set 80 msec. debouncing time. Default is 50 msec.
  button.setDebounceTicks(80);

#if defined (FSM_DEBUG)
  Serial.begin (115200);
#endif
} // setup

void loop()
{
 #if defined (FSM_DEBUG)
  if (curTime % 1000 == 0)
  {
    if (pri1 == false)
    {
      pri1 = true;
      Serial.print(F("\t PROGRESS:\t"));
      Serial.println(nextAction);
    }
  } else
  {
    pri1 = false;
  }
  // digitalWrite(ledSys, digitalRead(buttonUp));
#endif

  // put your main code here, to run repeatedly:
  curTime = millis();
  statusLED();
  button.tick(); // keep watching the push button:

  switch (nextAction)
  {
  case ACTION_OPEN:
    if (firstLong == true)
    {     statusLed = LED_FAST;
    } else {
          statusLed = LED_SLOW;
    }
    digitalWrite(motorPwm, HIGH);
    break;
  case ACTION_DISINFECTION:
    if (curTime > actionDutation)
    {
      nextAction = ACTION_IDLE;
      actionDutation = curTime + idleTime;
    }
    statusLed = LED_ON;
    if (digitalRead(buttonUp) == true)
    {nextAction = ACTION_OPEN;
    digitalWrite(motorPwm, HIGH);
    } else {
      digitalWrite(motorPwm, LOW);
    }
    break;
  case ACTION_IDLE:
    if (curTime > actionDutation)
    {
      nextAction = ACTION_DISINFECTION;
      actionDutation = curTime + uvcTime / 10;
    }
    statusLed = LED_SLOW;
    digitalWrite(motorPwm, HIGH);
    if (digitalRead(buttonUp) == true)
    {nextAction = ACTION_OPEN;}
    break;
  case ACTION_FORCE_WAIT:
    statusLed = LED_ON;
    digitalWrite(motorPwm, HIGH);
    break;
  case ACTION_FORCE_ON:
    statusLed = LED_FAST;
    digitalWrite(motorPwm, LOW);
    break;
  default:
    // printf("please select correct initial state");  // This should never occur
    break;
  }
}
