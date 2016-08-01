// LED imports.
#include <FastLED.h>

// Accel imports.
#include <Wire.h>
#include <Adafruit_LSM303_Old.h>

// Our custom data type.
#include "AccelReading.h"

/* Run parameters: */
#define MAX_BRIGHTNESS 0.75 // Max LED brightness.
#define MIN_BRIGHTNESS 0.3

/* Rain parameters */
#define RAIN_BRIGHTNESS 0.5

/* Neopixel parameters: */
#define LED_COUNT 40
#define DATA_PIN 6

/* Animation parameters: */
// ~15 ms minimum crawl speed for normal mode,
// ~2 ms minimum for superfast hack mode.
#define CRAWL_SPEED_MS 5
// General sensitivity of the animation.
// Raising this raises the vector magnitude needed to reach max (purple),
// and thus lowers sensitivity.
// Eg: 800 = more sensitive, 1600 = less sensitive
#define HERMES_SENSITIVITY 1600.0

// Emulate two strips by starting the crawl in the
// middle of the strip and crawling both ways.
#define ENABLE_SPLIT_STRIP 1
// Center LED, aka LED #0.
#define SPLIT_STRIP_CENTER 6

/* Sleeping parameters: */
#define SLEEP_BRIGHTNESS 0.30
#define SLEEP_CYCLE_MS 5000 // 5 second breathing cycle. Default: 5000
#define SLEEP_WAIT_TIME_MS 4000 // No movement for 5 seconds triggers breathing. Default: 5000
#define SLEEP_SENSITIVITY 100

/* Debug parameters: */
#define WAIT_FOR_KEYBOARD 0 // Use keyboard to pause/resume program.
#define PRINT_LOOP_TIME 0
#define PRINT_ACCEL_DATA 1
#define PRINT_SLEEP_TIME 0
#define PRINT_SLEEP_SENS 0

/* Advanced: */
#define ONBOARD_LED_PIN 7 // Pin D7 has an LED connected on FLORA.
#define ONBOARD_LED_NEOPIX 8 // Pin D8 has an LED connected on FLORA.

/* Button parameters: */
#define buttonPin 9
#define BUTTON_A_PIN 9
#define BUTTON_B_PIN 10
/* Number of total animations: */
#define NUMBER_OF_ANIMATIONS 3
#define NUMBER_OF_SLEEP_ANIMATIONS 2


CRGB onboard[1];
CRGB strip[LED_COUNT];

void setup() {
  // Initialize the strips
  FastLED.addLeds<NEOPIXEL, ONBOARD_LED_NEOPIX>(onboard, 1);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(strip, LED_COUNT);

  // Initialize the button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUTTON_PIN, HIGH);

  // Let the user know we are ready to start
  onboard[0] = CRGB::Blue;
  FastLED.show();

  // Initialize the accelerometer and blink when we are done
  accelSetup();
  calibrate();
}

///////////
// accel //
///////////

Adafruit_LSM303 lsm; // Bridge to accelerometer hardware.
AccelReading accelBuffer[10]; // Buffer for storing the last 10 readings.
int bufferPosition; // Current read position of the buffer.

double calibration; // Baseline for accelerometer data.
unsigned long calibrationLEDTime;
bool calibrationLEDOn;

// For breathing, track the time of the last significant movement.
unsigned long lastSignificantMovementTime;

// Initialization.

void accelSetup() {
  lsm.begin();
  bufferPosition = 0;

  // Initialize the full buffer to zero.
  for (int i = 0; i < bufferSize(); i++) {
    accelBuffer[i].x = 0;
    accelBuffer[i].y = 0;
    accelBuffer[i].z = 0;
  }
}

void calibrate() {
  calibration = 0;
  calibrationLEDTime = 0;
  calibrationLEDOn = false;

  showCalibration();

  while (1) {
    // Update onboard LED.
    unsigned long now = millis();
    if (now - calibrationLEDTime > 250) {
      calibrationLEDTime = now;
      calibrationLEDOn = !calibrationLEDOn;
      digitalWrite(ONBOARD_LED_PIN, calibrationLEDOn ? HIGH : LOW);
      if (calibrationLEDOn) {
        showCalibration();
      } else {
        showColorOff();
      }
    }

    // Fill the buffer.
    if (!fillBuffer()) {
      delay(10);
      continue;
    }

    // Check to see if we're done.
    bool pass = true;
    double avg = 0;
    for (int i = 0; i < bufferSize(); i++) {
      double m = getMagnitude(accelBuffer[i]);
      pass = pass && (abs(m - calibration) < 10);
      avg += m;
    }

    if (pass) {
      break;
    } else {
      avg /= bufferSize();
      calibration = avg;
    }
  }

  // Turn the calibration light off.
  digitalWrite(ONBOARD_LED_PIN, LOW);
}


void blinky(uint8_t c, uint8_t gHue) {
  for (int i = 1; i = c; i++) {
    for (int j = 0; j < i ; j++) {
      leds[j] = CHSV(gHue, 255, 192)
    }
    FastLED.show()

    fadeToBlackBy(leds, NUM_LEDS, 1)
  }
}

void buttons() {
  int b = checkButton();
  if (b == 1) {
    showType++;
    if (showType > NUMBER_OF_SLEEP_ANIMATIONS)
      showType = 1;
    blinky(showType, 255);
  }

  if (b == 2) {
    showButtonBType++;
    if (showButtonBType > NUMBER_OF_ANIMATIONS)
      showButtonBType = 1;
    blinky(showType, 255);
  }
  if (b == 3 || b == 4) {
    showButtonBType = 100;
    showType = 100;
  }
  //  buttonA();
  //  buttonB();
}


/*
  MULTI-CLICK: One Button, Multiple Events

  Oct 12, 2009
  Run checkButton() to retrieve a button event:
  Click
  Double-Click
  Hold
  Long Hold
*/

// Button timing variables
int debounce = 20; // ms debounce period to prevent flickering when pressing or releasing the button
int DCgap = 250; // max ms between clicks for a double click event
int holdTime = 2000; // ms hold period: how long to wait for press+hold event
int longHoldTime = 5000; // ms long hold period: how long to wait for press+hold event

// Other button variables
boolean buttonVal = HIGH; // value read from button
boolean buttonLast = HIGH; // buffered value of the button's previous state
boolean DCwaiting = false; // whether we're waiting for a double click (down)
boolean DConUp = false; // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true; // whether it's OK to do a single click
long downTime = -1; // time the button was pressed down
long upTime = -1; // time the button was released
boolean ignoreUp = false; // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false; // when held, whether to wait for the up event
boolean holdEventPast = false; // whether or not the hold event happened already
boolean longHoldEventPast = false;// whether or not the long hold event happened already

int checkButton()
{
  int event = 0;
  // Read the state of the button
  buttonVal = digitalRead(buttonPin);
  // Button pressed down
  if (buttonVal == LOW && buttonLast == HIGH && (millis() - upTime) > debounce) {
    downTime = millis();
    ignoreUp = false;
    waitForUp = false;
    singleOK = true;
    holdEventPast = false;
    longHoldEventPast = false;
    if ((millis() - upTime) < DCgap && DConUp == false && DCwaiting == true) DConUp = true;
    else DConUp = false;
    DCwaiting = false;
  }
  // Button released
  else if (buttonVal == HIGH && buttonLast == LOW && (millis() - downTime) > debounce) {
    if (not ignoreUp) {
      upTime = millis();
      if (DConUp == false) DCwaiting = true;
      else {
        event = 2;
        DConUp = false;
        DCwaiting = false;
        singleOK = false;
      }
    }
  }
  // Test for normal click event: DCgap expired
  if ( buttonVal == HIGH && (millis() - upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true) {
    event = 1;
    DCwaiting = false;
  }
  // Test for hold
  if (buttonVal == LOW && (millis() - downTime) >= holdTime) {
    // Trigger "normal" hold
    if (not holdEventPast) {
      event = 3;
      waitForUp = true;
      ignoreUp = true;
      DConUp = false;
      DCwaiting = false;
      //downTime = millis();
      holdEventPast = true;
    }
    // Trigger "long" hold
    if ((millis() - downTime) >= longHoldTime) {
      if (not longHoldEventPast) {
        event = 4;
        longHoldEventPast = true;
      }
    }
  }
  buttonLast = buttonVal;
  return event;
}
