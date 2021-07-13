/*
 SAW_controller

This is a sketch to operate circular saw. It offers manual control and automatic down move with automatic retract. 

Circuit setup:
* Inputs
  * Connect a push-buttons to command UP, DOWN, E-STOP to pins 6, 5, 3 (with optional 4,7 - 10 k pull-up resistor to VCC) and 1 - 2 k resistor from other side of button to GND for activation by low state.
  * Connect a push-buttons or other proximity sensor to detect UP end-stop and DOWN end-stop to pins 4, 2 (with optional 4,7 - 10 k pull-up resistor to VCC) and 1 - 2 k resistor from other side of button to GND for activation by low state.
  * Connect a potentiometer to set feed speed, wiper to pin A0 and ends of 4,7 - 50 k potentiometer between VCC nad GND.
* Outputs
  * Connect H bridge (like LM298) signals PWM (EN), CW, CCW signals to pins 9, 8, 7.
  * Optionally connect LEDs signaling motor direction to pins 11, 12. TODO: rework LEDs behavior
  * The pin 13 (ledSysPin) is used for status indication of Finite State Machine by blinking built-in LED.

We move saw UP and DOWN by pressing respective button.
If button is held for 1 second it will latch and we can release button.
When button is lathed we can unlatch it by pressing and releasing UP or Down button. TODO: rework to react to pressing.
When move is in automatic mode it will continue until it reach end-stop.
After hitting DOWN end-stop, saw will reverse and go UP.
After hitting UP end-stop, it will end cycle.
Feed is used for move DOWN. Move UP is always at full speed.
Feed speed (PWM) is set by potentiometer and can be changed at any time.
Motor is accelerated for 0.5 s on start of move, and stopped without deceleration.
E-STOP button stops motor, and ignore inputs until released.
 
 State-Diagram

 TODO: Split MoveUP and MoveDOWN to unlatched and latched states
 
                                      START               E-STOP button released
                                        |    +----------------------------------------------+
                                        V    V                                              |
                                     +-----------+                                    +-------------+        E-STOP
                                     |   IDLE    |<------------------------+          |    E-STOP   |<------ button
                      +--------------|           |-------+                 |          +-------------+        pressed
          DOWN button |              +-----------+       |                 |
            pressed   |                                  |                 |
                      V                                  |                 |
            +----------+                          +----------+             |
            |   Move   |                          |   Move   |             |
            |   DOWN   |                          |    UP    |             |
            +----------+                          +----------+             |
             |       ^                             |        ^              |
DOWN button  |       | UP or DOWN       UP button  |        | UP or DOWN   |
 longpress   |       |   button         longpress  |        |   button     |
             V       |  released                   V        |  released    |
            +----------+                          +----------+             |
            |   auto   |       DOWN end-stop      |   auto   |             |
            |   DOWN   |------------------------->|    UP    |-------------+
            +----------+                          +----------+ 
*/

#include <Arduino.h>

#include <EasyButton.h>
// #include <LogansGreatButton.h>
// #include <ObjectButton.h>

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
#define accelTime 500UL          // acceleration time 0,5s
#define autoPressDuration 1000UL // How long press for auto
unsigned long curTime = 0UL;     // will store current time to avoid multiple millis() calls
unsigned long accelEnd = 0UL;    // will store time when action should end
int feedValue = 0;
bool autoMode = false; // Is ato enabled
bool pri1 = false;

// The actions I ca do...
typedef enum
{
  ACTION_UP,   // move UP as long as UP Button is pressed
  ACTION_IDLE, // wait for button
  ACTION_DOWN, // move DOWN as long as DOWN Button is pressed
  ACTION_ESTOP // do not move
} MyActions;

MyActions nextAction = ACTION_IDLE; // no action when starting

// The actions I can do...
typedef enum
{
  LED_OFF,      // set LED "OFF".
  LED_ON,       // set LED "ON"
  LED_SLOW,     // blink LED "SLOW"
  LED_FAST,     // blink LED "FAST"
  LED_SUPERFAST // blink LED "SUPER FAST"
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
  case LED_SUPERFAST:
    // do a fast blinking
    if (curTime % 100 < 50)
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
  Serial.println(F("UP button released!"));
#endif
  switch (nextAction)
  {
  case ACTION_UP:
    nextAction = ACTION_IDLE;
    autoMode = false;
    break;
  case ACTION_DOWN:
    nextAction = ACTION_IDLE;
    autoMode = false;
    break;
  default:
    break;
  }
} // buttonUpOnPressedFunction

void buttonUpOnLongPressedFunction()
{
#if defined(FSM_DEBUG)
  Serial.println(F("UP Long button released!"));
#endif
  switch (nextAction)
  {
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
}

void buttonDownOnPressedFunction()
{
#if defined(FSM_DEBUG)
  Serial.println(F("DOWN button released!"));
#endif
  switch (nextAction)
  {
  case ACTION_UP:
    nextAction = ACTION_IDLE;
    autoMode = false;
    break;
  case ACTION_DOWN:
    nextAction = ACTION_IDLE;
    autoMode = false;
    break;
  default:
    break;
  }
} // buttonDownOnPressedFunction

void buttonDownOnLongPressedFunction()
{
#if defined(FSM_DEBUG)
  Serial.println(F("Down Long button released!"));
#endif
  switch (nextAction)
  {
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
}

void buttonEstopOnPressedFunction()
{
#if defined(FSM_DEBUG)
  Serial.println(F("ESTOP button released!"));
#endif
  nextAction = ACTION_IDLE;
} // buttonDownOnPressedFunction

void buttonEndstopUpPressedFunction()
{
#if defined(FSM_DEBUG)
  Serial.println(F("Endstop UP button released!"));
#endif
} // buttonDownOnPressedFunction

void buttonEndstopDowmPressedFunction()
{
#if defined(FSM_DEBUG)
  Serial.println(F("Endstop Down button released!"));
#endif
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

  // Initialize the button.
  buttonUp.begin();
  buttonDown.begin();
  buttonEstop.begin();
  buttonEndstopUp.begin();
  buttonEndstopDown.begin();

  // link the buttonUpOnPressedFunction function to be called on a click event.
  buttonUp.onPressed(buttonUpOnPressedFunction);
  buttonUp.onPressedFor(autoPressDuration, buttonUpOnLongPressedFunction);
  buttonDown.onPressed(buttonDownOnPressedFunction);
  buttonDown.onPressedFor(autoPressDuration, buttonDownOnLongPressedFunction);
  buttonEstop.onPressed(buttonEstopOnPressedFunction);
  buttonEndstopUp.onPressed(buttonEndstopUpPressedFunction);
  buttonEndstopDown.onPressed(buttonEndstopDowmPressedFunction);

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
      Serial.print(F("calculated PWM:\t"));
      Serial.println(feedValue);
      Serial.print(F("Auto:\t"));
      Serial.println(autoMode);
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

  buttonEstop.read();
  if (buttonEstop.isPressed() == true)
  {
    nextAction = ACTION_ESTOP;
#if defined(FSM_DEBUG)
    Serial.println("Estop Pressed!");
#endif
  }

  buttonUp.read();   // keep watching the push buttonUp:
  buttonDown.read(); // keep watching the push buttonUp:
  buttonEndstopUp.read();
  buttonEndstopDown.read();

  switch (nextAction)
  {
  case ACTION_ESTOP:
    digitalWrite(motorPwmPin, LOW);
    statusLed = LED_SUPERFAST;
    break;
  case ACTION_IDLE:
    autoMode = false;
    digitalWrite(motorPwmPin, LOW);
    statusLed = LED_SLOW;
    if (buttonUp.isPressed() == true)
    {
      nextAction = ACTION_UP;
      statusLed = LED_ON;
      accelEnd = curTime + accelTime;
#if defined(FSM_DEBUG)
      Serial.println("UP Button Pressed!");
#endif
    }
    if (buttonDown.isPressed() == true)
    {
      nextAction = ACTION_DOWN;
      statusLed = LED_ON;
      accelEnd = curTime + accelTime;
#if defined(FSM_DEBUG)
      Serial.println("DOWN Button Pressed!");
#endif
    }
    break;
  case ACTION_UP:
    if (buttonEndstopUp.isPressed() == false)
    {
      if (curTime < accelEnd)
      {
        feedValue = 255;
        // float scale = (curTime - (accelEnd - accelTime)) / accelTime;
        // feedValue = feedValue * scale;
        feedValue = map(curTime - (accelEnd - accelTime), 0, accelTime, 0, feedValue);
#if defined(FSM_DEBUG)
        Serial.println(feedValue);
#endif
      }
      else
      {
        // feedValue = map(analogRead(feedPin), 5, 4060, 0, 255);
        feedValue = 255;
      }
      digitalWrite(motorCwPin, LOW);
      digitalWrite(ledDowmPin, LOW);
      digitalWrite(motorCcwPin, HIGH);
      digitalWrite(ledUpPin, HIGH);
      analogWrite(motorPwmPin, feedValue); //PWM Speed Control
      // statusLed = LED_ON;
    }
    else
    {
#if defined(FSM_DEBUG)
      Serial.println(F("EndStop UP reached"));
#endif
      nextAction = ACTION_IDLE;
      digitalWrite(motorPwmPin, LOW);
    }
    break;
  case ACTION_DOWN:
    if (buttonEndstopDown.isPressed() == false)
    {
      if (curTime < accelEnd)
      {
        feedValue = map(analogRead(feedPin), 5, 4060, 0, 255);
        feedValue = map(curTime - (accelEnd - accelTime), 0, accelTime, 0, feedValue);
      }
      else
      {
        feedValue = map(analogRead(feedPin), 5, 4060, 0, 255);
      }
      digitalWrite(motorCwPin, HIGH);
      digitalWrite(ledDowmPin, HIGH);
      digitalWrite(motorCcwPin, LOW);
      digitalWrite(ledUpPin, LOW);
      analogWrite(motorPwmPin, feedValue); //PWM Speed Control
      // statusLed = LED_ON;
    }
    else
    {
#if defined(FSM_DEBUG)
      Serial.println(F("EndStop DOWN reached"));
#endif
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
  default:
    // printf("please select correct initial state");  // This should never occur
    break;
  }
}
