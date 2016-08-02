///////////
// sleep //
///////////

bool sleeping = false;
bool waiting = false;

bool sleep(double m) {
  int repeats = 0;
  unsigned long now = millis();

  if (abs(calibration - m) > SLEEP_SENSITIVITY) {
    lastSignificantMovementTime = now;
    waiting = false;
    sleeping = false;
    digitalWrite(ONBOARD_LED_PIN, sleeping ? HIGH : LOW);
    return false;
  } else {
    // Last significant movement time needs to be longer than sleep wait time.
    if ((now - lastSignificantMovementTime) < SLEEP_WAIT_TIME_MS) {
      // Haven't waited long enough.
      sleeping = false;
      digitalWrite(ONBOARD_LED_PIN, sleeping ? HIGH : LOW);
      return false;
    } else {
      sleeping = true;
      digitalWrite(ONBOARD_LED_PIN, sleeping ? HIGH : LOW);
      return true;
    }
    waiting = true;
  }
}

////////////
// wakeup //
////////////

bool wakeup() {
  accelPoll();
  double m = getMagnitude(getCurrentReading());

  if (abs(calibration - m) > SLEEP_SENSITIVITY) {
    return true;
  }
  return false;
}

