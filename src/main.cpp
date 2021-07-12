/*
 UVC-Timer.ino
 
 This is a sample sketch to operate chamber with Ultra Violet C... light for disinfection purposes. 
 
 Setup circuit:
 * Connect a pushbutton or hal sensor to pin A2 (motorPwmPin) and VCC, add 4,7 - 10 k resistor to GND for pulldown.
 * Connect relay control pin to pin A1 and UVC lamp to AC thru relay COM and NO terminals.
 * The pin 13 (ledSysPin) is used for output status of Finite State Machine by blinking built-in LED.
  
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
#include <EasyButton.h>
#include <LogansGreatButton.h>

// #define FSM_DEBUG
#undef FSM_DEBUG

// ---- PREPARE I/O
#define ledSysPin 13           // PB5 onboard LED
#define ledUpPin 12            // PB4 LED for UP movement
#define ledDowmPin 11          // PB3 LED for DOWN movement
#define buttonUpPin 6          // PD6 Command UP Button Pin
#define buttonDownPin 5        // PD5 Command DOWN Button Pin
#define buttonEndstopUpPin 4   // PD4 UP limit Button Pin
#define buttonEndstopDownPin 2 // PD2 DOWN limitButton Pin
#define buttonEstopPin 3       // PD3 emergeny stop Button Pin
#define motorPwmPin 9          // PB1 Control motor speed
#define motorCwPin 8           // PB0 Command motor directopn CW
#define motorCcwPin 7          // PB7 Command motor directopn CCW

// Define some defaults and controls
#if defined(FSM_DEBUG)
#define uvcTime 3000UL  // disinfection time 1m
#define idleTime 6000UL // idle time 30m
#else
#define uvcTime 1200000UL  // disinfection time 20m
#define idleTime 3600000UL // idle time 60m
#define doorTime 2000UL    // door seafty time
#endif
unsigned long curTime = 0UL;        // will store current time to avoid multiple millis() calls
unsigned long actionDutation = 0UL; // will store time when action should end
bool firstLong = false;             // first UvcTime cycle longer
bool pri1 = false;

// The actions I ca do...
typedef enum
{
  ACTION_UP,      // move UP as long as UP Button is pressed
  ACTION_CYCLE,   // move DOWN with feed to DOWN endstop
  ACTION_IDLE,    // wait for button
  ACTION_AUTO_UP, // move UP with max feed to UP endstop
  ACTION_DOWN,    // move DOWN as long as DOWN Button is pressed
  ACTION_ESTOP    // do not move
} MyActions;

MyActions nextAction = ACTION_IDLE; // no action when starting

// The actions I can do...
typedef enum
{
  LED_OFF,  // set LED "OFF".
  LED_ON,   // set LED "ON"
  LED_SLOW, // blink LED "SLOW"
  LED_FAST  // blink LED "FAST"
} LedState;

LedState statusLed = LED_SLOW; // Led blink on startup

// Setup a new OneButton on pin buttonUpPin.
// OneButton buttonUp(
//   buttonUpPin,     // Input pin for the button
//   HIGH,            // Button is active high
//   false            // Disable internal pull-up resistor
// );

EasyButton buttonUp(
    buttonUpPin, // Input pin for the button
    35,          // Debounce time
    true,        // Enable internal pullup resistor
    true         // Invert button logic. If true, low = pressed else high = pressed
);

EasyButton buttonDown(
    buttonDownPin, // Input pin for the button
    35,          // Debounce time
    true,        // Enable internal pullup resistor
    true         // Invert button logic. If true, low = pressed else high = pressed
);

void statusLED()
{
  switch (statusLed)
  {
  case LED_OFF:
    // Turn LED off
    digitalWrite(ledSysPin, LOW);
    break;
  case LED_ON:
    // turn LED on
    digitalWrite(ledSysPin, HIGH);
    break;
  case LED_SLOW:
    // do a slow blinking
    if (curTime % 1000 < 500)
    {
      digitalWrite(ledSysPin, LOW);
    }
    else
    {
      digitalWrite(ledSysPin, HIGH);
    } // if
    break;
  case LED_FAST:
    // do a fast blinking
    if (curTime % 200 < 100)
    {
      digitalWrite(ledSysPin, LOW);
    }
    else
    {
      digitalWrite(ledSysPin, HIGH);
    } // if
    break;
  default: // This should never occur
    break;
  }
}

// this function will be called when the buttonUp was pressed 1 time and them some time has passed.
void buttonUpOnPressedFunction()
{
#if defined(FSM_DEBUG)
  Serial.println(F("BUTTON!"));
#endif
  switch (nextAction)
  {
  case ACTION_IDLE:
    nextAction = ACTION_UP;
    break;
  case ACTION_DOWN:
    nextAction = ACTION_CYCLE;
    break;
  default:
    break;
  }
} // buttonUpOnPressedFunction

// this function will be called when the buttonDown was pressed 1 time and them some time has passed.
void buttonDownOnPressedFunction()
{
#if defined(FSM_DEBUG)
  Serial.println(F("BUTTON!"));
#endif
  switch (nextAction)
  {
  case ACTION_IDLE:
    nextAction = ACTION_DOWN;
    break;
  case ACTION_UP:
    nextAction = ACTION_AUTO_UP;
    break;
  default:
    break;
  }
} // buttonDownOnPressedFunction

// this function will be called when the buttonUp was pressed 2 times in a short timeframe.
void myDoubleClickFunction()
{
#if defined(FSM_DEBUG)
  Serial.println(F("BUTTON! DOUBLE"));
#endif
  switch (nextAction)
  {
  case ACTION_AUTO_UP:
    nextAction = ACTION_UP;
    break;
  case ACTION_DOWN:
    nextAction = ACTION_UP;
    break;
  case ACTION_UP:
    if (firstLong == true)
    {
      firstLong = false;
      nextAction = ACTION_AUTO_UP;
    }
    else
    {
      firstLong = true;
    }
    break;
  default:
    // This should never occur
    break;
  }
} // myDoubleClickFunction

// this function will be called when the buttonUp was pressed for one second.
void myLongPressFunction()
{
#if defined(FSM_DEBUG)
  Serial.println(F("BUTTON! LONG"));
#endif
  switch (nextAction)
  {
  case ACTION_UP:
    nextAction = ACTION_CYCLE;
    if (firstLong == true)
    {
      actionDutation = curTime + uvcTime * 2;
      firstLong = false;
    }
    else
    {
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
  pinMode(ledSysPin, OUTPUT);   // sets the digital pin as output
  pinMode(motorPwmPin, OUTPUT); // sets the digital pin as output

  // link the buttonUpOnPressedFunction function to be called on a click event.
  buttonUp.onPressed(buttonUpOnPressedFunction);
  buttonDown.onPressed(buttonDownOnPressedFunction);

  // link the doubleclick function to be called on a doubleclick event.
  // buttonUp.attachDoubleClick(myDoubleClickFunction);

  // link the longpress function to be called on a longpress event.
  // buttonUp.attachLongPressStart(myLongPressFunction);

  // set 80 msec. debouncing time. Default is 50 msec.
  // buttonUp.setDebounceTicks(80);

#if defined(FSM_DEBUG)
  Serial.begin(115200);
#endif
} // setup

void loop()
{
#if defined(FSM_DEBUG)
  if (curTime % 1000 == 0)
  {
    if (pri1 == false)
    {
      pri1 = true;
      Serial.print(F("\t PROGRESS:\t"));
      Serial.println(nextAction);
    }
  }
  else
  {
    pri1 = false;
  }
  // digitalWrite(ledSysPin, digitalRead(buttonUpPin));
#endif

  // put your main code here, to run repeatedly:
  curTime = millis();
  statusLED();
  buttonUp.read(); // keep watching the push buttonUp:

  switch (nextAction)
  {
  case ACTION_UP:
    if (firstLong == true)
    {
      statusLed = LED_FAST;
    }
    else
    {
      statusLed = LED_SLOW;
    }
    digitalWrite(motorPwmPin, HIGH);
    break;
  case ACTION_CYCLE:
    if (curTime > actionDutation)
    {
      nextAction = ACTION_IDLE;
      actionDutation = curTime + idleTime;
    }
    statusLed = LED_ON;
    if (digitalRead(buttonUpPin) == true)
    {
      nextAction = ACTION_UP;
      digitalWrite(motorPwmPin, HIGH);
    }
    else
    {
      digitalWrite(motorPwmPin, LOW);
    }
    break;
  case ACTION_IDLE:
    if (curTime > actionDutation)
    {
      nextAction = ACTION_CYCLE;
      actionDutation = curTime + uvcTime / 10;
    }
    statusLed = LED_SLOW;
    digitalWrite(motorPwmPin, HIGH);
    if (digitalRead(buttonUpPin) == true)
    {
      nextAction = ACTION_UP;
    }
    break;
  case ACTION_AUTO_UP:
    statusLed = LED_ON;
    digitalWrite(motorPwmPin, HIGH);
    break;
  case ACTION_DOWN:
    statusLed = LED_FAST;
    digitalWrite(motorPwmPin, LOW);
    break;
  default:
    // printf("please select correct initial state");  // This should never occur
    break;
  }
}
