#include <Arduino.h>
#include <Wire.h>
#include "config.hpp"

// #define KNOCK_SETUP
#define KNOCK_PLOT

void listen_knock_pattern();
bool check_knock_pattern();
bool knocked(uint8_t analog_pin, int threshold);

const int reject_diff = 25;  // If an individual knock is off by this percentage
                             // of a knock we don't unlock..
const int reject_avg_diff = 15;  // If the average timing of the knocks is
                                 // off by this percent we don't unlock.
const int knockFadeTime = 150;   // milliseconds we allow a knock to fade before
                                 // we listen for another one. (Debounce timer.)

const int maximumKnocks = 20;    // Maximum number of knocks to listen for.
const int knockComplete = 1200;  // Longest time to wait for a knock before we
                                 // assume that it's finished.

// Variables.
uint8_t secretKnockCount = 0;
uint16_t secretCode[maximumKnocks] = {
    500, 250, 250, 500, 1000, 500, 0, 0, 0, 0,
    0,   0,   0,   0,   0,    0,   0, 0, 0, 0};  // Initial setup: "Shave and a
                                                 // Hair Cut, two bits."

uint16_t knockReadings[maximumKnocks];  // When someone knocks this array fills
                                        // with delays between knocks.
int knockSensorValue = 0;               // Last reading of the knock sensor.
int programButtonPressed = false;  // Flag so we remember the programming button
                                   // setting at the end of the cycle.

void setup() {
  Serial.begin(115200);
  Serial.println("Setup");

  // precalculate secret code knocks count
  for (uint8_t i = 0; i < maximumKnocks; i++) {
    if (secretCode[i] > 0) {
      secretKnockCount++;
    }
  }

#ifdef KNOCK_SETUP
  pinMode(KNOCK_PIN2, INPUT);
  pinMode(KNOCK_PIN1, OUTPUT);
  digitalWrite(KNOCK_PIN1, LOW);

  unsigned long prev_time = 0;
  bool knocked = false;
  int max_reading = 0;

  while (1) {
    int sensorReading = analogRead(KNOCK_ANALOG_PIN);
    if (sensorReading > KNOCK_THRESH && !knocked) {
      prev_time = millis();
      knocked = true;
    } else if (sensorReading < KNOCK_THRESH && knocked) {
      Serial.println("fade time: " + String(millis() - prev_time));
      knocked = false;
    }
    if (sensorReading > max_reading) {
      max_reading = sensorReading;
    }
    if (sensorReading == 0 && max_reading != 0) {
      Serial.println("Max val: " + String(max_reading));
      // reset max reading
      max_reading = 0;
    }

    delay(1);
  }
#endif

#ifdef KNOCK_PLOT
  pinMode(KNOCK_PIN2, INPUT);
  pinMode(KNOCK_PIN1, OUTPUT);
  digitalWrite(KNOCK_PIN1, LOW);

  while (1) {
    int sensorReading = analogRead(KNOCK_ANALOG_PIN);
    Serial.println(sensorReading);
    delay(1);
  }
#endif
}

void loop() {
  if (knocked(KNOCK_ANALOG_PIN, KNOCK_THRESH)) {
    Serial.println("Start listening");
    delay(knockFadeTime);
    listen_knock_pattern();

    if (check_knock_pattern()) {
      Serial.println("sequence OK!");
    } else {
      Serial.println("invalid sequence");
    }

    // Reset incoming sequence array
    for (int i = 0; i < maximumKnocks; i++) {
      // Serial.print(String(knockReadings[i]) + ",");
      knockReadings[i] = 0;
    }
    Serial.println();
  }
}

// Records the timing of knocks.
void listen_knock_pattern() {
  unsigned long startTime = millis();  // Reference for when this knock started.
  unsigned long now = 0;
  uint8_t currentKnockNumber = 0;  // Incrementer for the array.

  // timeout or maximum knocks reached?
  while ((millis() - startTime < knockComplete) &&
         (currentKnockNumber < maximumKnocks)) {
    if (knocked(KNOCK_ANALOG_PIN, KNOCK_THRESH)) {
      // new knock
      now = millis();
      knockReadings[currentKnockNumber] = now - startTime;
      startTime = now;

      // increment the counter
      currentKnockNumber++;

      delay(knockFadeTime);
    }
  }

  // reset counter
  currentKnockNumber = 0;
}

// Sees if our knock matches the secret.
// returns true if it's a good knock, false if it's not.
// todo: break it into smaller functions for readability.
bool check_knock_pattern() {
  // simplest check first: Did we get the right number of knocks?
  uint8_t currentKnockCount = 0;
  uint16_t maxKnockInterval = 0;  // We use this later to normalize the times.

  for (uint8_t i = 0; i < maximumKnocks; i++) {
    Serial.print(String(knockReadings[i]) + ",");
    if (knockReadings[i] > 0) {
      currentKnockCount++;
    }

    if (knockReadings[i] > maxKnockInterval) {
      // collect normalization data while we're looping.
      maxKnockInterval = knockReadings[i];
    }
  }

  // check knocks number
  Serial.println(currentKnockCount);
  if (currentKnockCount != secretKnockCount) {
    Serial.println("Incorrect count");
    return false;
  }

  /*  Now we compare the relative intervals of our knocks, not the absolute time
     between them. (ie: if you do the same pattern slow or fast it should still
     open the door.) This makes it less picky, which while making it less secure
     can also make it less of a pain to use if you're tempo is a little slow or
     fast.
  */
  int total_diff = 0;
  int interval_diff = 0;

  for (uint8_t i = 0; i < maximumKnocks; i++) {
    // Intervals normalization
    knockReadings[i] = map(knockReadings[i], 0, maxKnockInterval, 0, 1000);

    // calculate interval diff
    interval_diff = abs(knockReadings[i] - secretCode[i]);

    // check single interval diff
    if (interval_diff > reject_diff * 10) {
      Serial.println("Incorrect interval");

      return false;
    }
    total_diff += interval_diff;
  }

  // check average interval diff
  if ((total_diff / secretKnockCount) > reject_avg_diff * 10) {
    Serial.println("Incorrect average");

    return false;
  }

  return true;
}

bool knocked(uint8_t analog_pin, int threshold) {
  pinMode(KNOCK_PIN2, INPUT);
  pinMode(KNOCK_PIN1, OUTPUT);
  digitalWrite(KNOCK_PIN1, LOW);

  int sensorReading = analogRead(analog_pin);

  // if the sensor reading is greater than the threshold:
  if (sensorReading >= threshold) {
    return true;
  }

  return false;
}

/*
void btn_sound() {
  isPaused = true;
  // Save starting time to wait after sound end without using delay()
  prev_sound = millis();

  toneAC(NOTE_E6, 5, NOTE_DURATION, true);  // it was 500
}
*/