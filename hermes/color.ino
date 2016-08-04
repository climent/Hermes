/////////////////////
// Color functions //
/////////////////////

void fadeToColor(uint32_t c, uint8_t wait) {
  boolean done true;
  for (i = 0; i < NUM_LEDS, i++) {
    done = true;
    if (leds[i] != c) {
      if (leds[i] > c) {
        leds[i] -= 1;
        done = false;
      } else {
        leds[i] += 1;
        done = false
      }
    }
    delay(wait);
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  // TODO(): c is 32-bit color for Neopixel lib. We need to convert it to 0-255 values.
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(c, 255, MAX_BRIGHTNESS * 255);
    FastLED.show();
    delay(wait);
  }
}

void blinky(uint8_t c, uint8_t gHue) {
  for (int i = 1; i = c; i++) {
    for (int j = 0; j < i ; j++) {
      if (blinker) {
        leds[j] = CHSV(gHue, 255, 192);
      } else {
        fadeToBlackBy(leds, NUM_LEDS, 100);
      }
    }
    FastLED.show();
    delay(500);
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

void theaterChase(CRGB* leds, uint8_t num_leds, bool rainbow) {
  for (int i = 0; i < num_leds; i = i + 3) {
    if (i + cycle < num_leds) {
      if (rainbow == true) {
        leds[i + cycle] = CHSV(gHue + i, 255, 192);
      } else {
        leds[i + cycle] = CRGB::White;
      }
    }
    if (i + cycle - 1 >= 0 && i + cycle - 1 < num_leds ) {
      leds[i + cycle - 1] = CRGB::Black;
    }
    if (i + cycle - 2 >= 0 && i + cycle - 2 < num_leds ) {
      leds[i + cycle - 2] = CRGB::Black;
    }
  }
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

