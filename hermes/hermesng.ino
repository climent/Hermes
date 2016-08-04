// LED imports.
#include <FastLED.h>

// Accel imports.
#include <Wire.h>
#include <Adafruit_LSM303.h>

// Our custom data type.
#include "AccelReading.h"

/* Run parameters: */
#define MAX_BRIGHTNESS 0.75 // Max LED brightness.
#define MIN_BRIGHTNESS 0.3

/* Rain parameters */
#define RAIN_BRIGHTNESS 0.5

/* Neopixel parameters: */
#define NUM_LEDS 40
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



CRGB onboard[1];
CRGB strip[NUM_LEDS];

// Whether to show the blinking lead
boolean blinker true;
uint8_t gHue = 0;
uint8_t chase = 0;

void setup() {
  // Initialize the strips
  FastLED.addLeds<NEOPIXEL, ONBOARD_LED_NEOPIX>(onboard, 1);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(strip, NUM_LEDS);

  // Initialize the button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUTTON_PIN, HIGH);

  // Let the user know we are ready to start
  onboard[0] = CRGB::Blue;
  FastLED.show();

  // Initialize the accelerometer
  accelSetup();
  calibrate();

  // Turn off the calibration pixel
  onboard[0] = CRGB::Black;
  FastLED.show();
}

/* Number of total animations: */
#define A_ANIMATIONS 3
#define B_ANIMATIONS 2

int fadeout = 0;
int fadein = 0;

void loop() {
  EVERY_N_MILLISECONDS( 20 ) {
    gHue++;  // slowly cycle the "base color" through the rainbow
  }
  EVERY_N_MILLISECONDS( 100 ) {
    blinker = !blinker;
    fadeout--;
    if (fadeout < 0) {
      fadeout = 0;
    };
    fadein++;
    if (fadein > 255) {
      fadein = 255;
    };
    chase++;
    if (chase == 3) {
      chase = 0;
    }
  }

  accelPoll();
  updateLEDs();
  FastLED.show();
  buttons();
}

void updateLEDs() {
  // Largest vector needed to hit max color (1.0).
  double upperBound = HERMES_SENSITIVITY;
  double magnitude = getMagnitude(getCurrentReading());

  if (!sleep(magnitude)) {
    showAnimation(a_animation);
  } else {
    double normalizedVector = abs(calibration - magnitude);
    double scale = normalizedVector / upperBound;
    uint32_t pixelColor = pixelColorForScale(scale);
    showSleep(b_animation);
  }
}

void showAnimation(int a_animation) {
  switch (a_animation) {
    case 1:
      theaterChase(leds, NUM_LEDS, true);
      break;
    case 2:
      theaterChase(leds, NUM_LEDS, true);
      break;
    case 3:
      theaterChase(leds, NUM_LEDS, false);
      break;
    case 4:
      fill_rainbow(leds, NUM_LEDS, gHue, 5);
      break;
    case 100:
      fadeToBlackBy(leds, NUM_LEDS, 5);
      break;
  }
}

void showSleep(int b_animation) {
  switch (b_animation) {
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    case 100:
      fadeToBlackBy(leds, NUM_LEDS, 5);
      break;
    default:
      break;
  }
}
