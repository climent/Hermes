// LED wire.
#include <Adafruit_NeoPixel.h>
// Our custom data type.
#include "AccelReading.h"

//Adafruit_NeoPixel onboard_strip = Adafruit_NeoPixel(1, 6, NEO_GRB + NEO_KHZ800);

#define LED_COUNT 144
#define DATA_PIN 6

int COLOR_RANGE = 384;
uint32_t lastColor;
unsigned long lastCrawl;
uint32_t lightArray[LED_COUNT];

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, DATA_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  // put your setup code here, to run once:
  //onboard_strip.begin();
  //onboard_strip.show();

  checkSuperfastHack();
  colorSetup();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  crawlColor(strip.Color(0, 0, 0));
  if (millis() % 50 == 0) {
    crawlColor(strip.Color(20, 20, 20));
    crawlColor(strip.Color(0, 0, 0));
    crawlColor(strip.Color(20, 20, 20));
    crawlColor(strip.Color(0, 0, 0));
    crawlColor(strip.Color(20, 20, 20));
    crawlColor(strip.Color(0, 0, 0));
    crawlColor(strip.Color(20, 20, 20));
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
  
  for (int i = 0; i < LED_COUNT; i++) {
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

  // Shift the array.
  for (int i = LED_COUNT - 1; i > 0; --i) {
    lightArray[i] = lightArray[i - 1];
  }

  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, lightArray[i]);
  }
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

