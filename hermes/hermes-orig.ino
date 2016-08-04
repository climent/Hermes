//// Fill the dots one after the other with a color
//void colorWipe(uint32_t c, uint8_t wait) {
//  for (uint16_t i = 0; i < onboard_pixel.numPixels(); i++) {
//    onboard_pixel.setPixelColor(i, c);
//    onboard_pixel.show();
//    delay(wait);
//  }
//}
//
//uint32_t Wheel(byte WheelPos) {
//  // All the values are multiplied by .5, or otherwise we overload the board on USB power and it crashes hard.
//  if (WheelPos < 85) {
//    return strip.Color(.5 * (WheelPos * 3), .5 * (255 - WheelPos * 3), 0);
//  } else if (WheelPos < 170) {
//    WheelPos -= 85;
//    return strip.Color(.5 * (255 - WheelPos * 3), 0, .5 * (WheelPos * 3));
//  } else {
//    WheelPos -= 170;
//    return strip.Color(0, .5 * (WheelPos * 3), .5 * (255 - WheelPos * 3));
//  }
//}
//
/////////////
//// color //
/////////////
//
//int COLOR_RANGE = 384;
//uint32_t lastColor;
//unsigned long lastCrawl;
//uint32_t lightArray[LED_COUNT];
//bool goingToSleep = false;
//
//void colorSetup() {
//  lastColor = 0;
//  lastCrawl = 0;
//
//  // Turn the strip on.
//  strip.begin();
//  stripShow();
//
//  // Initialize the LED buffer.
//  for (int i = 0; i < LED_COUNT; i++) {
//    lightArray[i] = 0;
//  }
//}
//
//// Changes the colors of the strip, from the current value to the given value.
//void fadeOut(int red, int green, int blue, int wait) {
//  while (1) {
//    bool timeToGo = true;
//    for (int i = 0; i < strip.numPixels(); i++) {
//      uint8_t *p,
//              r = (uint8_t)(lightArray[i] >> 16),
//              g = (uint8_t)(lightArray[i] >>  8),
//              b = (uint8_t)lightArray[i];
//      if (r > red) {
//        r -= 1;
//        timeToGo = false;
//      } else if (r < red) {
//        r += 1;
//        timeToGo = false;
//      }
//      if (g > green) {
//        g -= 1;
//        timeToGo = false;
//      } else if (g < green) {
//        g += 1;
//        timeToGo = false;
//      }
//      if (b > blue) {
//        b -= 1;
//        timeToGo = false;
//      } else if (b < blue) {
//        b += 1;
//        timeToGo = false;
//      }
//      if (timeToGo) {
//        return;
//      }
//      lightArray[i] = strip.Color(r, g, b);
//      strip.setPixelColor(i, lightArray[i]);
//    }
//    stripShow();
//    delay(wait);
//    buttons();
//    if (wakeup()) {
//      return;
//    }
//  }
//}
//
//void crawlColorStrip(uint32_t color) {
//  // Set the head pixel to the new color.
//  uint32_t head = lightArray[0];
//  lightArray[0] = color;
//
//  unsigned long now = millis();
//
//  // Shift the array if it's been long enough since last shifting,
//  // or if a new color arrives.
//  bool shouldUpdate =
//    (now - lastCrawl > CRAWL_SPEED_MS)
//    || (color != head);
//
//  if (!shouldUpdate) {
//    return;
//  }
//
//  lastCrawl = now;
//
//  // Shift the array.
//  for (int i = LED_COUNT - 1; i > 0; --i) {
//    lightArray[i] = lightArray[i - 1];
//  }
//
//  for (int i = 0; i < LED_COUNT; i++) {
//    strip.setPixelColor(i, lightArray[i]);
//  }
//  stripShow();
//}
//
//void vuMeterPower(uint32_t color) {
//  for (int i = 0; i < LED_COUNT; i++) {
//    lightArray[i] = color;
//    strip.setPixelColor(i, lightArray[i]);
//  }
//  stripShow();
//}
//
//// "Crawls" the given color along the strip.
//// This always sets LED[0] to the given color.
//// After CRAWL_SPEED_MS milliseconds,
//// we set LED[n + 1] = LED[n] for each LED.
//void crawlColor(uint32_t color) {
//  // Set the head pixel to the new color.
//  uint32_t head = lightArray[0];
//  lightArray[0] = color;
//
//  unsigned long now = millis();
//
//  // Shift the array if it's been long enough since last shifting,
//  // or if a new color arrives.
//  bool shouldUpdate =
//    (now - lastCrawl > CRAWL_SPEED_MS)
//    || (color != head);
//
//  if (!shouldUpdate) {
//    return;
//  }
//
//  lastCrawl = now;
//
//  // Shift the array.
//  for (int i = LED_COUNT - 1; i > 0; --i) {
//    lightArray[i] = lightArray[i - 1];
//  }
//
//  if (ENABLE_SPLIT_STRIP) {
//    int centerLED = SPLIT_STRIP_CENTER;
//    int LEDsPerSide = floor(LED_COUNT / 2);
//
//    // Crawl 'low' side (center down)
//    uint32_t *pixelColor = lightArray;
//    for (int led = centerLED - 1; led >= centerLED - 1 - LEDsPerSide; led--) {
//      strip.setPixelColor(constrainBetween(led, 0, LED_COUNT - 1), *pixelColor++);
//    }
//
//    // Crawl 'high' side (center up)
//    pixelColor = lightArray;
//    for (int led = centerLED; led < centerLED + LEDsPerSide; led++) {
//      strip.setPixelColor(constrainBetween(led, 0, LED_COUNT - 1), *pixelColor++);
//    }
//    stripShow();
//    return;
//  }
//
//  for (int i = 0; i < LED_COUNT; i++) {
//    strip.setPixelColor(i, lightArray[i]);
//  }
//  stripShow();
//}
//
//int constrainBetween(int value, int lower, int higher) {
//  if (value < lower) {
//    value = higher - (lower - value) + 1;
//  } else if (value > higher) {
//    value = lower + (value - higher) - 1;
//  }
//  return value;
//}
//
//// Sets the strip all one color.
//// Scale parameter is a value 0.0 to 1.0,
//// representing how far on the rainbow to go.
//void showColor(float scale) {
//  uint32_t pixelColor = pixelColorForScale(scale);
//
//  if (pixelColor == lastColor) {
//    // No change since we last set the pixels; don't bother changing them.
//    return;
//  }
//  lastColor = pixelColor;
//
//  // Serial.print("Show "); Serial.print(scale); Serial.println(c);
//  for (int i = 0; i < LED_COUNT; i++) {
//    strip.setPixelColor(i, pixelColor);
//  }
//  stripShow();
//}
//
//
//// Color 1 from 384; brightness 0.0 to 1.0.
//uint32_t color(uint16_t color, float brightness)  {
//  byte r, g, b;
//  int range = color / 128;
//  switch (range) {
//    case 0: // Red to Yellow (1 to 128)
//      r = 127 - color % 128;
//      g = color % 128;
//      b = 0;
//      break;
//    case 1: // Yellow to Teal (129 to 256)
//      r = 0;
//      g = 127 - color % 128;
//      b = color % 128;
//      break;
//    case 2: // Teal to Purple (257 to 384)
//      r = color % 128;
//      g = 0;
//      b = 127 - color % 128;
//      break;
//  }
//  r *= brightness;
//  g *= brightness;
//  b *= brightness;
//  return strip.Color(r, g, b);
//}
//
//void showColorOff() {
//  colorOff();
//  stripShow();
//}
//
//void colorOff() {
//  for (int i = 0; i < strip.numPixels(); i++) {
//    strip.setPixelColor(i, 0);
//  }
//}
//
//// Show the calibration colors.
//void showCalibration() {
//  colorOff();
//
//  int mid = LED_COUNT / 2;
//  float brightness = 0.3;
//
//  // Red
//  strip.setPixelColor(mid - 1, strip.Color(127 * brightness, 0, 0));
//  // Green
//  strip.setPixelColor(mid, strip.Color(0, 127 * brightness, 0));
//  // Blue
//  strip.setPixelColor(mid + 1, strip.Color(0, 0,  127 * brightness));
//
//  stripShow();
//}
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
//
//void rotation() {
//  crawlColor(strip.Color(20, 20, 20));
//  crawlColor(strip.Color(18, 18, 18));
//  crawlColor(strip.Color(12, 12, 12));
//  crawlColor(strip.Color(10, 10, 10));
//  crawlColor(strip.Color(7, 7, 7));
//  crawlColor(strip.Color(6, 6, 6));
//  crawlColor(strip.Color(4, 4, 4));
//  crawlColor(strip.Color(2, 2, 2));
//  crawlColor(strip.Color(1, 1, 1));
//  for (int i = 0; i < (strip.numPixels() / 2) - 9; i++) {
//    crawlColor(strip.Color(0, 0, 0));
//    buttons();
//  }
//}
//
//void rotationFullStrip() {
//  crawlColorStrip(strip.Color(20, 20, 20));
//  crawlColorStrip(strip.Color(18, 18, 18));
//  crawlColorStrip(strip.Color(12, 12, 12));
//  crawlColorStrip(strip.Color(10, 10, 10));
//  crawlColorStrip(strip.Color(7, 7, 7));
//  crawlColorStrip(strip.Color(6, 6, 6));
//  crawlColorStrip(strip.Color(4, 4, 4));
//  crawlColorStrip(strip.Color(2, 2, 2));
//  crawlColorStrip(strip.Color(1, 1, 1));
//  for (int i = 0; i < (strip.numPixels() ) - 9; i++) {
//    crawlColorStrip(strip.Color(0, 0, 0));
//    buttons();
//  }
//}
//
////////////
//// rain //
////////////
//
//void rain() {
//  crawlColor(strip.Color(0, 0, 0));
//  if (millis() % 50 == 0) {
//    //crawlColor(strip.Color(0, 0, 0));
//    crawlColor(strip.Color(20, 20, 20));
//    crawlColor(strip.Color(18, 18, 18));
//    crawlColor(strip.Color(12, 12, 12));
//    crawlColor(strip.Color(10, 10, 10));
//    crawlColor(strip.Color(7, 7, 7));
//    crawlColor(strip.Color(6, 6, 6));
//    crawlColor(strip.Color(4, 4, 4));
//    crawlColor(strip.Color(2, 2, 2));
//    crawlColor(strip.Color(1, 1, 1));
//  }
//  buttons();
//}
//
///////////////
//// breathe //
///////////////
//
//void breathe() {
//  for (int i = 0; i < 100; i++) {
//    strip.setPixelColor(i, strip.Color(3, 0, 0));
//    if (wakeup()) {
//      return;
//    }
//    stripShow();
//    delay(10);
//    buttons();
//  }
//  fadeOut(24, 0, 0, 20);
//  fadeOut(3, 0, 0, 20);
//}
