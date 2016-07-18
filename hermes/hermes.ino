/*
   Hermes LED shoes
   Copyright 2013-2014 RGAM LLC
*/

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
#define NUMBER_OF_SLEEP_ANIMATIONS 4

///////////////////////////////////////////////////////////////////

// LED imports.
#include <Adafruit_NeoPixel.h>

// Accel imports.
#include <Wire.h>
#include <Adafruit_LSM303_Old.h>

// Our custom data type.
#include "AccelReading.h"

Adafruit_NeoPixel onboard_pixel = Adafruit_NeoPixel(1, ONBOARD_LED_NEOPIX, NEO_GRB + NEO_KHZ800);
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
  onboard_pixel.begin();
  onboard_pixel.show();

  if (WAIT_FOR_KEYBOARD) {
    Serial.println("Strip is ready.");
  }

  // During the setup process, signal where we are with colors:
  // Blue: waiting for the accelerator to calibrate.

  checkSuperfastHack();
  colorSetup();

  colorWipe(onboard_pixel.Color(0, 0, 150), 300); // Blue

  accelSetup();

  if (WAIT_FOR_KEYBOARD) {
    Serial.println("Acceleration done.");
  }

  // Blinky when we are done.
  colorWipe(onboard_pixel.Color(255, 255, 255), 500); // White
  colorWipe(onboard_pixel.Color(255, 0, 0), 100); // Red
  colorWipe(onboard_pixel.Color(0, 0, 0), 100); // Black
  colorWipe(onboard_pixel.Color(0, 255, 0), 100); // Green
  colorWipe(onboard_pixel.Color(0, 0, 0), 100); // Black
  colorWipe(onboard_pixel.Color(255, 0, 0), 100); // Red
  colorWipe(onboard_pixel.Color(0, 0, 0), 100); // Black
  colorWipe(onboard_pixel.Color(0, 255, 0), 100); // Green
  colorWipe(onboard_pixel.Color(0, 0, 0), 100); // Red

  // Setup 2 buttons
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_B_PIN, INPUT_PULLUP);

}

// Main loop
void loop() {
  buttons();
  loopDebug();
  accelPoll();
  updateLED();
}


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

int  showButtonBType = 1;
int showType = 1;


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < onboard_pixel.numPixels(); i++) {
    onboard_pixel.setPixelColor(i, c);
    onboard_pixel.show();
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


//////////////
// vu-meter //
//////////////
//
//#define TOP       (LED_COUNT +1)
//#define DC_OFFSET  0  // DC offset in mic signal - if unusure, leave 0
//
//int
//  lvl       = 0,     // Current "dampened" audio level
//  minLvlAvg = 0,      // For dynamic adjustment of graph low & high
//  maxLvlAvg = 512;
//
//void vuMeter(double scale) {
////
////  float brightness = MAX_BRIGHTNESS * (scale + MIN_BRIGHTNESS);
////  int c = COLOR_RANGE * scale; // Intentionally round to an int.
////
////  uint8_t  i;
////  uint16_t minLvl, maxLvl;
//  int      n, height;
////
//  n   = abs(scale - 512 - DC_OFFSET); // Center on zero
////  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
//  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)
////
//  // Calculate bar height based on dynamic min/max levels (fixed point):
//  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
//  Serial.println(height);
//  if(height < 0L)       height = 0;      // Clip output
//  else if(height > TOP) height = TOP;
////  if(height > peak)     peak   = height; // Keep 'peak' dot at top
////
////
////  // Color pixels based on rainbow gradient
//  for(int i=0; i<strip.numPixels(); i++) {
//    if(i >= height)               strip.setPixelColor(i,   0,   0, 0);
//    else strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));
//  }
//  strip.show();
//
////  // Draw peak dot
////  if(peak > 0 && peak <= N_PIXELS-1) strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
////    strip.show(); // Update strip
////    // Every few frames, make the peak pixel drop by 1:
////    if(++dotCount >= PEAK_FALL) { //fall rate
////      if(peak > 0) peak--;
////      dotCount = 0;
////    }
////
////    vol[volCount] = n;                      // Save sample for dynamic leveling
////    if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
////
////    // Get volume range of prior frames
////    minLvl = maxLvl = vol[0];
////    for(i=1; i<SAMPLES; i++) {
////      if(vol[i] < minLvl)      minLvl = vol[i];
////      else if(vol[i] > maxLvl) maxLvl = vol[i];
////    }
////    // minLvl and maxLvl indicate the volume range over prior frames, used
////  // for vertically scaling the output graph (so it looks interesting
////  // regardless of volume level).  If they're too close together though
////  // (e.g. at very low volume levels) the graph becomes super coarse
////  // and 'jumpy'...so keep some minimum distance between them (this
////  // also lets the graph go to zero when no sound is playing):
////  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
////  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
////  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
////
//}

uint32_t Wheel(byte WheelPos) {
  // All the values are multiplied by .5, or otherwise we overload the board on USB power and it crashes hard.
  if (WheelPos < 85) {
    return strip.Color(.5 * (WheelPos * 3), .5 * (255 - WheelPos * 3), 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(.5 * (255 - WheelPos * 3), 0, .5 * (WheelPos * 3));
  } else {
    WheelPos -= 170;
    return strip.Color(0, .5 * (WheelPos * 3), .5 * (255 - WheelPos * 3));
  }
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

// Rainbow Cycle Program - Equally distributed
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 ; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      lightArray[i] = Wheel(((i * 256 / strip.numPixels()) + j) & 255);
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    if (wakeup()) return;
    buttons();
    stripShow();
    delay(wait);
  }
}

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
  double magnitude = getMagnitude(getCurrentReading());

  // Change LED strip color.
  if (sleep(magnitude)) {
    switch (showType) {
      case 1:
        fadeOut(3, 0, 0, 20);
        breathe();
        break;
      case 2:
        rain();
        break;
      case 3:
        rotation();
        break;
      case 4:
        rotationFullStrip();
        break;
      case 100:
        fadeOut(0, 0, 0, 100);
        break;
      // In case we missed the animation, use this as default
      default:
        fadeOut(3, 0, 0, 20);
        breathe();
        break;
    }
  } else {
    double normalizedVector = abs(calibration - magnitude);
    double scale = normalizedVector / upperBound;
    uint32_t pixelColor = pixelColorForScale(scale);
    printAccelData(scale);

    switch (showButtonBType) {
      case 1:
        crawlColor(pixelColor);
        break;
      case 2:
        crawlColorStrip(pixelColor);
        break;
      case 3:
        vuMeterPower(pixelColor);
        break;
      case 4:
        rainbowCycle(0);
        break;
      case 100:
        fadeOut(0, 0, 0, 100);
        break;
      default:
        crawlColor(pixelColor);
        break;
    }
  }
}

// Changes the colors of the strip, from the current value to the given value.
void fadeOut(int red, int green, int blue, int wait) {
  while (1) {
    bool timeToGo = true;
    for (int i = 0; i < strip.numPixels(); i++) {
      uint8_t *p,
              r = (uint8_t)(lightArray[i] >> 16),
              g = (uint8_t)(lightArray[i] >>  8),
              b = (uint8_t)lightArray[i];
      if (r > red) {
        r -= 1;
        timeToGo = false;
      } else if (r < red) {
        r += 1;
        timeToGo = false;
      }
      if (g > green) {
        g -= 1;
        timeToGo = false;
      } else if (g < green) {
        g += 1;
        timeToGo = false;
      }
      if (b > blue) {
        b -= 1;
        timeToGo = false;
      } else if (b < blue) {
        b += 1;
        timeToGo = false;
      }
      if (timeToGo) {
        return;
      }
      lightArray[i] = strip.Color(r, g, b);
      strip.setPixelColor(i, lightArray[i]);
    }
    stripShow();
    delay(wait);
    buttons();
    if (wakeup()) {
      return;
    }
  }
}

void crawlColorStrip(uint32_t color) {
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

  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, lightArray[i]);
  }
  stripShow();
}

void vuMeterPower(uint32_t color) {
  for (int i = 0; i < LED_COUNT; i++) {
    lightArray[i] = color;
    strip.setPixelColor(i, lightArray[i]);
  }
  stripShow();
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

//// Shows the color progression.
//void showColorProgression() {
//  for (int j = 0; j < 384; j++) {
//    for (int i = 0; i < strip.numPixels(); i++) {
//      strip.setPixelColor(i, color(j, 0.5));
//    }
//    stripShow();
//    delay(1);
//    if (wakeup()) {
//      return;
//    }
//  }
//  for (int i = 0; i < strip.numPixels(); i++) {
//    strip.setPixelColor(i, 0);
//  }
//  stripShow();
//  delay(1);
//}

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

void rotation() {
  crawlColor(strip.Color(20, 20, 20));
  crawlColor(strip.Color(18, 18, 18));
  crawlColor(strip.Color(12, 12, 12));
  crawlColor(strip.Color(10, 10, 10));
  crawlColor(strip.Color(7, 7, 7));
  crawlColor(strip.Color(6, 6, 6));
  crawlColor(strip.Color(4, 4, 4));
  crawlColor(strip.Color(2, 2, 2));
  crawlColor(strip.Color(1, 1, 1));
  for (int i = 0; i < (strip.numPixels() / 2) - 9; i++) {
    crawlColor(strip.Color(0, 0, 0));
    buttons();
  }
}

void rotationFullStrip() {
  crawlColorStrip(strip.Color(20, 20, 20));
  crawlColorStrip(strip.Color(18, 18, 18));
  crawlColorStrip(strip.Color(12, 12, 12));
  crawlColorStrip(strip.Color(10, 10, 10));
  crawlColorStrip(strip.Color(7, 7, 7));
  crawlColorStrip(strip.Color(6, 6, 6));
  crawlColorStrip(strip.Color(4, 4, 4));
  crawlColorStrip(strip.Color(2, 2, 2));
  crawlColorStrip(strip.Color(1, 1, 1));
  for (int i = 0; i < (strip.numPixels() ) - 9; i++) {
    crawlColorStrip(strip.Color(0, 0, 0));
    buttons();
  }
}

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
  buttons();
}

/////////////
// breathe //
/////////////

void buttons() {
  int b = checkButton();
  if (b == 1) {
    showType++;
    if (showType > NUMBER_OF_SLEEP_ANIMATIONS)
      showType = 1;
    for (int i = 0; i < 3; i++) {
      showColorOff();
      delay(100);
      showCalibration();
      delay(100);
    }
  }

  if (b == 2) {
    showButtonBType++;
    if (showButtonBType > NUMBER_OF_ANIMATIONS)
      showButtonBType = 1;
    for (int i = 0; i < 7; i++) {
      showColorOff();
      delay(100);
      showCalibration();
      delay(100);
    }
  }
  if (b == 3 || b == 4) {
    showButtonBType = 100;
    showType = 100;
  }
//  buttonA();
//  buttonB();
}

void breathe() {
  for (int i = 0; i < 100; i++) {
    strip.setPixelColor(i, strip.Color(3, 0, 0));
    if (wakeup()) {
      return;
    }
    stripShow();
    delay(10);
    buttons();
  }
  fadeOut(24, 0, 0, 20);
  fadeOut(3, 0, 0, 20);
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

bool sleep(double m) {
  int repeats = 0;
  unsigned long now = millis();

  //  // See if this movement is significant, aka enough to wake us from sleep.
  //  double m = getMagnitude(getCurrentReading());

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
