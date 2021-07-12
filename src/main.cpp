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

#define FSM_DEBUG
// #undef FSM_DEBUG

// ---- PREPARE I/O
#define buttonUpPin 6          // PD6 Command UP Button Pin
#define buttonDownPin 5        // PD5 Command DOWN Button Pin
#define buttonEndstopUpPin 4   // PD4 UP limit Button Pin
#define buttonEndstopDownPin 2 // PD2 DOWN limitButton Pin
#define buttonEstopPin 3       // PD3 emergeny stop Button Pin
#define feedPin A0             // PC0 feed speed potentiometer
#define ledSysPin 13           // PB5 onboard LED
#define ledUpPin 12            // PB4 LED for UP movement
#define ledDowmPin 11          // PB3 LED for DOWN movement
#define motorPwmPin 9          // PB1 Control motor speed
#define motorCwPin 8           // PB0 Command motor directopn CW
#define motorCcwPin 7          // PB7 Command motor directopn CCW

// Define some defaults and controls
#if defined(FSM_DEBUG)
#define accelTime 500UL    // acceleration time 0,5s
#define doorTime 2000UL    // door seafty time
#else
#define accelTime 500UL    // acceleration time 0,5s
#define doorTime 2000UL    // door seafty time
#endif
unsigned long curTime = 0UL;  // will store current time to avoid multiple millis() calls
unsigned long accelEnd = 0UL; // will store time when action should end
int feedValue = 0;
bool autoMode = false; // Is ato enabled
bool pri1 = false;

// The actions I ca do...
typedef enum
{
  ACTION_UP,      // move UP as long as UP Button is pressed
  ACTION_IDLE,    // wait for button
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
    35,            // Debounce time
    true,          // Enable internal pullup resistor
    true           // Invert button logic. If true, low = pressed else high = pressed
);

EasyButton buttonEndstopUp(
    buttonEndstopUpPin, // Input pin for the button
    35,                 // Debounce time
    true,               // Enable internal pullup resistor
    true                // Invert button logic. If true, low = pressed else high = pressed
);

EasyButton buttonEndstopDown(
    buttonEndstopDownPin, // Input pin for the button
    35,                   // Debounce time
    true,                 // Enable internal pullup resistor
    true                  // Invert button logic. If true, low = pressed else high = pressed
);

EasyButton buttonEstop(
    buttonEstopPin, // Input pin for the button
    35,             // Debounce time
    true,           // Enable internal pullup resistor
    true            // Invert button logic. If true, low = pressed else high = pressed
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

// these functions will be called when the button is released.
void buttonUpOnPressedFunction()
{
#if defined(FSM_DEBUG)
  Serial.println(F("UP button released!!"));
#endif
  switch (nextAction)
  {
  case ACTION_UP:
    nextAction = ACTION_IDLE;
    break;
  case ACTION_DOWN:
    autoMode = true;
    statusLed = LED_FAST;
    #if defined(FSM_DEBUG)
      Serial.println(F("Auto ON"));
    #endif
    break;
  default:
    break;
  }
} // buttonUpOnPressedFunction

void buttonDownOnPressedFunction()
{
#if defined(FSM_DEBUG)
  Serial.println(F("DOWN button released!"));
#endif
  switch (nextAction)
  {
  case ACTION_DOWN:
    nextAction = ACTION_IDLE;
    break;
  case ACTION_UP:
    autoMode = true;
    statusLed = LED_FAST;
    #if defined(FSM_DEBUG)
      Serial.println(F("Auto ON"));
    #endif
    break;
  default:
    break;
  }
} // buttonDownOnPressedFunction

void buttonEstopOnPressedFunction()
{
#if defined(FSM_DEBUG)
  Serial.println(F("ESTOP button released!"));
#endif
  nextAction = ACTION_IDLE;
} // buttonDownOnPressedFunction

void setup()
{
  // put your setup code here, to run once:
  pinMode(ledSysPin, OUTPUT);   // sets the digital pin as output
  pinMode(ledUpPin, OUTPUT);    // sets the digital pin as output
  pinMode(ledDowmPin, OUTPUT);  // sets the digital pin as output
  pinMode(motorPwmPin, OUTPUT); // sets the digital pin as output
  pinMode(motorCwPin, OUTPUT);  // sets the digital pin as output
  pinMode(motorCcwPin, OUTPUT); // sets the digital pin as output
  pinMode(feedPin, INPUT);      // declares pin A0 as input

  // link the buttonUpOnPressedFunction function to be called on a click event.
  buttonUp.onPressed(buttonUpOnPressedFunction);
  buttonDown.onPressed(buttonDownOnPressedFunction);
  buttonEstop.onPressed(buttonEstopOnPressedFunction);

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
  buttonUp.read();   // keep watching the push buttonUp:
  buttonDown.read(); // keep watching the push buttonUp:
  buttonEndstopUp.read();
  buttonEndstopDown.read();
  buttonEstop.read();

  if (buttonEstop.isPressed() == true)
  {
    nextAction = ACTION_ESTOP;
  }

  switch (nextAction)
  {
  case ACTION_IDLE:
    autoMode = false;
    digitalWrite(motorPwmPin, LOW);
    statusLed = LED_SLOW;
    if (buttonUp.isPressed() == true)
    {
      nextAction = ACTION_UP;
      accelEnd = curTime + accelTime;
    }
    if (buttonDown.isPressed() == true)
    {
      nextAction = ACTION_DOWN;
      accelEnd = curTime + accelTime;
    }
    break;
  case ACTION_UP:
    if (buttonEndstopUp.isPressed() == false)
    {
      if (curTime > accelEnd)
      {
        feedValue = map(analogRead(feedPin), 0, 1023, 0, 255);
      }
      else
      {
        feedValue = map(analogRead(feedPin), 0, 1023, 0, 255); // TODO: Acceleration logic
      }
      #if defined(FSM_DEBUG)
        Serial.println(F("calculated PWM!"));
        Serial.println(feedValue);
      #endif
      digitalWrite(motorCwPin, LOW);
      digitalWrite(motorCcwPin, HIGH);
      analogWrite(motorPwmPin, feedValue); //PWM Speed Control
      statusLed = LED_ON;
    }
    else
    {
      nextAction = ACTION_IDLE;
      digitalWrite(motorPwmPin, LOW);
    }
    break;
  case ACTION_DOWN:
    if (buttonEndstopDown.isPressed() == false)
    {
      if (curTime > accelEnd)
      {
        feedValue = map(analogRead(feedPin), 0, 1023, 0, 255);
      }
      else
      {
        feedValue = map(analogRead(feedPin), 0, 1023, 0, 255); // TODO: Acceleration logic
      }
      digitalWrite(motorCwPin, HIGH);
      digitalWrite(motorCcwPin, LOW);
      analogWrite(motorPwmPin, feedValue); //PWM Speed Control
      statusLed = LED_ON;
    }
    else
    {
      if (autoMode == false)
      {
        nextAction = ACTION_IDLE;
        digitalWrite(motorPwmPin, LOW);
      }
      else
      {
        nextAction = ACTION_UP;
        accelEnd = curTime + accelTime;
      }
    }
    break;
  case ACTION_ESTOP:
    break;
  default:
    // printf("please select correct initial state");  // This should never occur
    break;
  }
}
