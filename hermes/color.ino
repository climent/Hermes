//////////////////
// Color functions
//////////////////

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
        leds[j] = CRGB::Black;
      }
    }
    FastLED.show();
    delay(500);
  }
}
