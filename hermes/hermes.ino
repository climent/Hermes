/*
   Hermes LED shoes
   Copyright 2013-2014 RGAM LLC
*/

/* Constants */

const uint8_t KEYFRAMES[]  = {
  3, 3, 3, 3, 3, 4, 4 ,5 ,5 ,6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12,
  13, 13, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22,
  23, 23, 24, 24,
  // Rising
//  22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22,
//  22, 22, 22, 22, 22, 22, 22, 24, 26, 28, 31, 34, 37, 40, 42, 46, 50, 55, 60,
//  65, 70, 75, 80, 85, 90, 95, 100, 105, 112, 121, 131, 141, 151, 161, 171, 171,

  // Falling
//  161, 151, 141, 131, 121, 112, 105, 100, 95, 90, 85, 80, 75, 70, 65, 60, 55,
//  50, 45, 42, 40, 37, 34, 31, 28, 26, 24, 22, 22, 22, 22, 22, 22, 22, 22, 22,
//  22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22,
//  22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22,
//  22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22,
//  22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22,
};

/* Run parameters: */
#define MAX_BRIGHTNESS 0.75 // Max LED brightness.
#define MIN_BRIGHTNESS 0.3

/* Rain parameters */
#define RAIN_BRIGHTNESS 0.5

/* Neopixel parameters: */
#define LED_COUNT 144
#define DATA_PIN 6

/* Animation parameters: */
// ~15 ms minimum crawl speed for normal mode,
// ~2 ms minimum for superfast hack mode.
#define CRAWL_SPEED_MS 2
// General sensitivity of the animation.
// Raising this raises the vector magnitude needed to reach max (purple),
// and thus lowers sensitivity.
// Eg: 800 = more sensitive, 1600 = less sensitive
#define HERMES_SENSITIVITY 1600.0

// Emulate two strips by starting the crawl in the
// middle of the strip and crawling both ways.
#define ENABLE_SPLIT_STRIP 1
// Center LED, aka LED #0.
#define SPLIT_STRIP_CENTER 72

/* Sleeping parameters: */
#define SLEEP_BRIGHTNESS 0.30
#define SLEEP_CYCLE_MS 5000 // 5 second breathing cycle. Default: 5000
#define SLEEP_WAIT_TIME_MS 4000 // No movement for 5 seconds triggers breathing. Default: 5000
#define SLEEP_SENSITIVITY 100

/* Debug parameters: */
#define WAIT_FOR_KEYBOARD 0 // Use keyboard to pause/resume program.
#define PRINT_LOOP_TIME 0
#define PRINT_ACCEL_DATA 0
#define PRINT_SLEEP_TIME 0
#define PRINT_SLEEP_SENS 0

/* Advanced: */
#define ONBOARD_LED_PIN 7 // Pin D7 has an LED connected on FLORA.
#define ONBOARD_LED_NEOPIX 8 // Pin D8 has an LED connected on FLORA.

/* The sleep mode to use: */
#define SLEEP_MODE "breathe"

///////////////////////////////////////////////////////////////////

// LED imports.
#include <Adafruit_NeoPixel.h>

// Accel imports.
#include <Wire.h>
#include <Adafruit_LSM303_Old.h>

// Our custom data type.
#include "AccelReading.h"

Adafruit_NeoPixel onboard_strip = Adafruit_NeoPixel(1, ONBOARD_LED_NEOPIX, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, DATA_PIN, NEO_GRB + NEO_KHZ800);


void setup() {
  if (WAIT_FOR_KEYBOARD) {
    Serial.begin(9600);

    // Wait for serial to initalize.
    while (!Serial) { }

    Serial.println("Strike any key to start...");

    // Wait for the next keystroke.
    while (!Serial.available()) { }

    // Clear the serial buffer.
    Serial.read();

    Serial.println("Ready to roll!");
  }

  // Let's signal when we are ready to configure the board.
  onboard_strip.begin();
  onboard_strip.show();

  if (WAIT_FOR_KEYBOARD) {
    Serial.println("Strip is ready.");
  }

  // During the setup process, signal where we are with colors:
  // Blue: waiting for the accelerator to calibrate. 

  checkSuperfastHack();
  colorSetup();

  // set this to 128 to avoind the 144 led strip from hanging when testing
  strip.setBrightness(128);

  colorWipe(onboard_strip.Color(0, 0, 255), 300); // Blue

  accelSetup();

  if (WAIT_FOR_KEYBOARD) {
    Serial.println("Acceleration done.");
  }

  // Blinky when we are done.
  colorWipe(onboard_strip.Color(255, 255, 255), 500); // White
  colorWipe(onboard_strip.Color(255, 0, 0), 100); // Red
  colorWipe(onboard_strip.Color(0, 0, 0), 100); // Black
  colorWipe(onboard_strip.Color(0, 255, 0), 100); // Green
  colorWipe(onboard_strip.Color(0, 0, 0), 100); // Black
  colorWipe(onboard_strip.Color(255, 0, 0), 100); // Red
  colorWipe(onboard_strip.Color(0, 0, 0), 100); // Black
  colorWipe(onboard_strip.Color(0, 255, 0), 100); // Green
  colorWipe(onboard_strip.Color(0, 0, 0), 100); // Red
  pinMode(9, INPUT_PULLUP);
}

bool keypressed = true;


// Main loop
void loop() {
  // This code can be used to add a button to the board and have a change in behavior
  // E.g., start doing a light loop
  // E.g., put the board in sleep mode and stop polling until the button is pressed again
  //unsigned long now = millis();

  //if (digitalRead(9) == LOW)  {
  //  unsigned long pressed = millis();
  //  if (now - pressed > 250){
  //    keypressed = !keypressed;
  //  }
  //} 

  //Serial.println(String(keypressed));

  loopDebug();
  accelPoll();
  updateLED();
  
  //if (keypressed) {
  //} else {
  //  crawlColor(strip.Color(0, 0, 0));
  //  if (millis() % 50 == 0) {
  //    crawlColor(strip.Color(20, 20, 20));
  //    crawlColor(strip.Color(0, 0, 0));
  //    crawlColor(strip.Color(20, 20, 20));
  //    crawlColor(strip.Color(0, 0, 0));
  //    crawlColor(strip.Color(20, 20, 20));
  //    crawlColor(strip.Color(0, 0, 0));
  //    crawlColor(strip.Color(20, 20, 20));
  //  }
}


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < onboard_strip.numPixels(); i++) {
    onboard_strip.setPixelColor(i, c);
    onboard_strip.show();
    delay(wait);
  }
}

// Debug functions controlled by run/debug parameters.
unsigned long before = 0;
void loopDebug() {
  if (WAIT_FOR_KEYBOARD) {
    pauseOnKeystroke();
  }
  if (PRINT_LOOP_TIME) {
    unsigned long now = millis();
    Serial.println(now - before);
    before = millis();
  }
}

void checkSuperfastHack() {
#if SUPERFAST_LED_HACK
#ifdef _COMPILE_TIME_LEDS_
  Serial.println("Using superfast LED hack.");
#elif
  // Wait for serial to initalize.
  while (!Serial) { }
  Serial.println("WARNING: You need to install the LPD8806Fast library.");
#endif
#endif
}

void pauseOnKeystroke() {
  if (Serial.available()) {
    // Clear the serial buffer.
    Serial.read();

    Serial.println("Paused. Strike any key to resume...");

    // Turn all LEDs off.
    showColorOff();

    // Wait for the next keystroke.
    while (!Serial.available()) { }

    // Clear the serial buffer.
    Serial.read();
  }
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

///////////
// accel //
///////////

Adafruit_LSM303_Old lsm; // Bridge to accelerometer hardware.
AccelReading accelBuffer[10]; // Buffer for storing the last 10 readings.
int bufferPosition; // Current read position of the buffer.

double calibration; // Baseline for accelerometer data.
unsigned long calibrationLEDTime;
bool calibrationLEDOn;

// For breathing, track the time of the last significant movement.
unsigned long lastSignificantMovementTime;

// Initialization.
void accelSetup() {
  //Serial.println("Initializing accel setup...");

  lsm.begin();

  bufferPosition = 0;

  // Initialize the full buffer to zero.
  for (int i = 0; i < bufferSize(); i++) {
    accelBuffer[i].x = 0;
    accelBuffer[i].y = 0;
    accelBuffer[i].z = 0;
  }

  calibrate();
}

void calibrate() {
  Serial.println("Initializing accel calibration...");
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
      if (WAIT_FOR_KEYBOARD) {
        Serial.print("Calibration: ");
        Serial.println(calibration);
      }
      break;
    } else {
      avg /= bufferSize();
      calibration = avg;
    }
  }

  // Turn the calibration light off.
  digitalWrite(ONBOARD_LED_PIN, LOW);
}

// Gathers data from accelerometer into the buffer. Only writes to the buffer
// if the hardware has gathered data since we last wrote to the buffer.
void accelPoll() {
  // Read new accelerometer data. If there is no new data, return immediately.
  if (!fillBuffer()) {
    return;
  }

  /* PRINT DATA: */
  // printBuffer();
  // printDelta();
  // printMagnitude();
  // Serial.println();
}

// Gets the vector for the given reading.
double getVector(AccelReading reading) {
  double normalizedVector = abs(calibration - getMagnitude(reading));
  return normalizedVector;
}

///////////////////////////////////////////////////////////////////

// This may or may not fill the next buffer position. If the accelerometer hasn't
// processed a new reading since the last buffer, this function immediately exits,
// returning false.
// Otherwise, if the accelerometer has read new data, this function advances the
// buffer position, fills the buffer with accelerometer data, and returns true.
bool fillBuffer() {
  // Read from the hardware.
  lsm.read();

  AccelReading newReading;
  newReading.x = lsm.accelData.x;
  newReading.y = lsm.accelData.y;
  newReading.z = lsm.accelData.z;

  // The accelerometer hasn't processed a new reading since the last buffer.
  // Do nothing and return false.
  if (equalReadings(getCurrentReading(), newReading)) {
    return false;
  }

  // The accelerometer has read new data.

  // Advance the buffer.
  if (++bufferPosition >= bufferSize()) {
    bufferPosition = 0;
  }

  AccelReading *mutableCurrentReading = &accelBuffer[bufferPosition];

  mutableCurrentReading->x = newReading.x;
  mutableCurrentReading->y = newReading.y;
  mutableCurrentReading->z = newReading.z;

  return true;
}

///////////////////////////////////////////////////////////////////

// Gets the average difference between the latest buffer and previous buffer.
int getDelta() {
  AccelReading previousReading = getPreviousReading();
  AccelReading currentReading  = getCurrentReading();

  int deltaX = abs(abs(currentReading.x) - abs(previousReading.x));
  int deltaY = abs(abs(currentReading.y) - abs(previousReading.y));
  int deltaZ = abs(abs(currentReading.z) - abs(previousReading.z));

  return (deltaX + deltaY + deltaZ) / 3;
}

void printDelta() {
  AccelReading previousReading = getPreviousReading();
  AccelReading currentReading  = getCurrentReading();

  int deltaX = abs(abs(currentReading.x) - abs(previousReading.x));
  int deltaY = abs(abs(currentReading.y) - abs(previousReading.y));
  int deltaZ = abs(abs(currentReading.z) - abs(previousReading.z));

  Serial.print(deltaX); Serial.print ("\t");
  Serial.print(deltaY); Serial.print ("\t");
  Serial.print(deltaZ); Serial.print ("\t");
  Serial.print(getDelta()); Serial.println();
}

// Gets the vector magnitude for the given reading.
// http://en.wikipedia.org/wiki/Euclidean_vector#Length
double getMagnitude(AccelReading reading) {
  double x = reading.x;
  double y = reading.y;
  double z = reading.z;

  double vector = x * x + y * y + z * z;

  return sqrt(vector);
}

void printMagnitude() {
  Serial.println(getMagnitude(getCurrentReading()));
}

// Prints the latest buffer reading to the screen.
void printBuffer() {
  Serial.print(accelBuffer[bufferPosition].x); Serial.print ("\t");
  Serial.print(accelBuffer[bufferPosition].y); Serial.print ("\t");
  Serial.print(accelBuffer[bufferPosition].z); Serial.println();
}

///////////////////////////////////////////////////////////////////

// Returns the number of items held by the buffer.
int bufferSize() {
  return sizeof(accelBuffer) / sizeof(accelBuffer[0]);
}

AccelReading getCurrentReading() {
  return accelBuffer[bufferPosition];
}

// Gets the previous buffer reading.
AccelReading getPreviousReading() {
  int previous = bufferPosition - 1;
  if (previous < 0) previous = bufferSize() - 1;
  return accelBuffer[previous];
}

// Returns true if two readings are equal.
bool equalReadings(AccelReading a, AccelReading b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}


///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

///////////
// color //
///////////

int COLOR_RANGE = 384;
uint32_t lastColor;
unsigned long lastCrawl;
uint32_t lightArray[LED_COUNT];
bool goingToSleep = false;

void colorSetup() {
  lastColor = 0;
  lastCrawl = 0;

  // Turn the strip on.
  strip.begin();
  stripShow();

  // Initialize the LED buffer.
  for (int i = 0; i < LED_COUNT; i++) {
    lightArray[i] = 0;
  }
}

void printAccelData(double scale) {
  if (PRINT_ACCEL_DATA) {
    String accel_data = "#";
    for (int n = 1; n < scale * 100 / 1.5 ; n++) {
      accel_data += "#";
    }
    Serial.println(accel_data);
  }
}

void updateLED() {
  // LED color takes a value from 0.0 to 1.0. Calculate scale from the current vector.

  // Largest vector needed to hit max color (1.0).
  double upperBound = HERMES_SENSITIVITY;
  double normalizedVector = abs(calibration - getMagnitude(getCurrentReading()));
  double scale = normalizedVector / upperBound;
  uint32_t pixelColor = pixelColorForScale(scale);

  printAccelData(scale);

  // Change LED strip color.
//  if (goingToSleep) {
//    GoToSleep(color);
//¨  }
  if (sleep()) {
    if (SLEEP_MODE == "breathe") {
      decreaseColor();
      breathe();
    } else if (SLEEP_MODE == "rain") {
      decreaseColor();
      rain();
    } else {
      decreaseColor();
      breathe();
    }
  } else {
    //Serial.println(scale);
    //Serial.println(pixelColor);
    //showColorProgression();
    crawlColor(pixelColor);
  }
}

// decreases the colors of the strip, from the current value to the given value.
// It uses final 

void decreaseColor() {
  while (1) {
    bool timeToGo = true;
    Serial.println("Decreasing colors...");
    for (int i = 0; i < strip.numPixels(); i++) {
      uint8_t *p,
      r = (uint8_t)(lightArray[i] >> 16),
      g = (uint8_t)(lightArray[i] >>  8),
      b = (uint8_t)lightArray[i];
      if (r > 3) {
        r -= 1;
        Serial.println("Drecreasing red");
        timeToGo = false;
      }
      if (g > 3) {
        g -= 1;
        Serial.println("Drecreasing green");
        timeToGo = false;
      }
      if (b > 3) {
        b -= 1;
        Serial.println("Drecreasing blue");
        timeToGo = false;
      }
      if (timeToGo) {
        return;
      }
      lightArray[i] = strip.Color(r, g, b);
      strip.setPixelColor(i, lightArray[i]);
    }
    stripShow();
    //delay(500);
    if (wakeup()){
      return;
    }
  }  
}

// "Crawls" the given color along the strip.
// This always sets LED[0] to the given color.
// After CRAWL_SPEED_MS milliseconds,
// we set LED[n + 1] = LED[n] for each LED.
void crawlColor(uint32_t color) {

  // Set the head pixel to the new color.
  uint32_t head = lightArray[0];
  lightArray[0] = color;

  unsigned long now = millis();

  // Shift the array if it's been long enough since last shifting,
  // or if a new color arrives.
  bool shouldUpdate =
    (now - lastCrawl > CRAWL_SPEED_MS)
    || (color != head);

  if (!shouldUpdate) {
    return;
  }

  lastCrawl = now;

  // Shift the array.
  for (int i = LED_COUNT - 1; i > 0; --i) {
    lightArray[i] = lightArray[i - 1];
  }

  if (ENABLE_SPLIT_STRIP) {
    int centerLED = SPLIT_STRIP_CENTER;
    int LEDsPerSide = floor(LED_COUNT / 2);

    // Crawl 'low' side (center down)
    uint32_t *pixelColor = lightArray;
    for (int led = centerLED - 1; led >= centerLED - 1 - LEDsPerSide; led--) {
      strip.setPixelColor(constrainBetween(led, 0, LED_COUNT - 1), *pixelColor++);
    }

    // Crawl 'high' side (center up)
    pixelColor = lightArray;
    for (int led = centerLED; led < centerLED + LEDsPerSide; led++) {
      strip.setPixelColor(constrainBetween(led, 0, LED_COUNT - 1), *pixelColor++);
    }

    stripShow();

    return;
  }

  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, lightArray[i]);
  }
  stripShow();
}

int constrainBetween(int value, int lower, int higher) {
  if (value < lower) {
    value = higher - (lower - value) + 1;
  } else if (value > higher) {
    value = lower + (value - higher) - 1;
  }
  return value;
}

// Sets the strip all one color.
// Scale parameter is a value 0.0 to 1.0,
// representing how far on the rainbow to go.
void showColor(float scale) {
  uint32_t pixelColor = pixelColorForScale(scale);

  if (pixelColor == lastColor) {
    // No change since we last set the pixels; don't bother changing them.
    return;
  }
  lastColor = pixelColor;

  // Serial.print("Show "); Serial.print(scale); Serial.println(c);
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, pixelColor);
  }
  stripShow();
}

// Returns a pixel color for use by strip.setPixelColor().
// Automatically adjusts brightness.
// Takes a scale, from 0.0 to 1.0, indicating progression
// through the color rainbow.
uint32_t pixelColorForScale(double scale) {
  float brightness = MAX_BRIGHTNESS * (scale + MIN_BRIGHTNESS);
  int c = COLOR_RANGE * scale; // Intentionally round to an int.

  return color(c, brightness);
}

// Shows the color progression.
void showColorProgression() {
  for (int j = 0; j < 384; j++) {
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, color(j, 0.5));
    }
    stripShow();
    delay(1);
  }

  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, 0);
  }
  stripShow();
  delay(1);
}

// Color 1 from 384; brightness 0.0 to 1.0.
uint32_t color(uint16_t color, float brightness)  {
  byte r, g, b;
  int range = color / 128;
  switch (range) {
    case 0: // Red to Yellow (1 to 128)
      r = 127 - color % 128;
      g = color % 128;
      b = 0;
      break;
    case 1: // Yellow to Teal (129 to 256)
      r = 0;
      g = 127 - color % 128;
      b = color % 128;
      break;
    case 2: // Teal to Purple (257 to 384)
      r = color % 128;
      g = 0;
      b = 127 - color % 128;
      break;
  }
  r *= brightness;
  g *= brightness;
  b *= brightness;
  return strip.Color(r, g, b);
}

void showColorOff() {
  colorOff();
  stripShow();
}

void colorOff() {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, 0);
  }
}

// Show the calibration colors.
void showCalibration() {
  colorOff();

  int mid = LED_COUNT / 2;
  float brightness = 0.3;

  // Red
  strip.setPixelColor(mid - 1, strip.Color(127 * brightness, 0, 0));
  // Green
  strip.setPixelColor(mid, strip.Color(0, 127 * brightness, 0));
  // Blue
  strip.setPixelColor(mid + 1, strip.Color(0, 0,  127 * brightness));

  stripShow();
}

void stripShow() {
#if SUPERFAST_LED_HACK
#ifdef _COMPILE_TIME_LEDS_
  // These settings are for Leonardo (ATmega32u4) with
  // LED pins data=6, clock=12.
  // See CompileTimeLEDs.h for more info.
  strip.showCompileTime<6, 7>(PORTD, PORTD);
#elif
  // Can't actually use superfast hack, it isn't installed properly.
  strip.show();
#endif
  return;
#endif

  strip.show();
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

//////////
// rain //
//////////

void rain() {
  crawlColor(strip.Color(0, 0, 0));
  if (millis() % 50 == 0) {
    //crawlColor(strip.Color(0, 0, 0));
    crawlColor(strip.Color(20, 20, 20));
    crawlColor(strip.Color(18, 18, 18));
    crawlColor(strip.Color(12, 12, 12));
    crawlColor(strip.Color(10, 10, 10));
    crawlColor(strip.Color(7, 7, 7));
    crawlColor(strip.Color(6, 6, 6));
    crawlColor(strip.Color(4, 4, 4));
    crawlColor(strip.Color(2, 2, 2));
    crawlColor(strip.Color(1, 1, 1));
  }

}

/////////////
// breathe //
/////////////

void breathe() {
  int keyframePointer = 0;
  int numKeyframes = sizeof(KEYFRAMES) - 1;

  for (int keyframePointer = 0; keyframePointer < numKeyframes; keyframePointer++) {
    for (int i = 0; i < strip.numPixels(); i++) {
      uint8_t color = KEYFRAMES[keyframePointer];
      //(SLEEP_BRIGHTNESS * 127 * KEYFRAMES[keyframePointer]) / 256;
      strip.setPixelColor(i, strip.Color(color, 0, 0));
      lightArray[i] = strip.Color(color, 0, 0);
    }
    if (wakeup()) {
      break;
    }
    stripShow();
    delay(20);
  }
  decreaseColor();
}

////////////
// wakeup //
////////////

bool wakeup() {
  accelPoll();
  double m = getMagnitude(getCurrentReading());
  
  if (abs(calibration - m) > SLEEP_SENSITIVITY) {
    if (PRINT_SLEEP_SENS) {
      Serial.println("Waking up...");
    }
    return true;
  }
  return false;
}

///////////
// sleep //
///////////

bool sleeping = false;
bool waiting = false;

bool sleep() {
  int repeats = 0;
  unsigned long now = millis();

  // See if this movement is significant, aka enough to wake us from sleep.
  
  double m = getMagnitude(getCurrentReading());
  
  if (abs(calibration - m) > SLEEP_SENSITIVITY) {
    lastSignificantMovementTime = now;
    waiting = false;
    repeats += 1;
    // Print only 1 every 4 cycles, to avoid overflowing the console.
    if (PRINT_SLEEP_SENS && (repeats % 4 == 0)) {
      Serial.println(abs(calibration - m));
    }
    sleeping = false;
    digitalWrite(ONBOARD_LED_PIN, sleeping ? HIGH : LOW);
    return false;
  } else {
    if (PRINT_SLEEP_SENS) {
      if (!waiting) {
        Serial.println("Waiting to sleep...");
      }
    }
    // Last significant movement time needs to be longer than sleep wait time.
    if ((now - lastSignificantMovementTime) < SLEEP_WAIT_TIME_MS) {
      // Haven't waited long enough.
      if (PRINT_SLEEP_TIME) {
        Serial.println(" wait period > " + String(now - lastSignificantMovementTime) + " / " + String(SLEEP_WAIT_TIME_MS));
        //Serial.println(SLEEP_WAIT_TIME_MS);
      }
      sleeping = false;
      digitalWrite(ONBOARD_LED_PIN, sleeping ? HIGH : LOW);
      return false;
    } else {
      if (!sleeping && PRINT_SLEEP_SENS) {
        Serial.println("We are now sleeping...");
      }
      sleeping = true;
      digitalWrite(ONBOARD_LED_PIN, sleeping ? HIGH : LOW);
      return true;    
    }
    waiting = true;
  }

}

///////////////////////////////////////////////////////////////////

// double normalizedVector = abs(calibration - getMagnitude(getCurrentReading()));
