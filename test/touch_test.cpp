#include <Arduino.h>
#include <CapacitiveSensor.h>
#include <Wire.h>
#include "config.hpp"

// high number of sample lead to a better resolution, but with slower response
#define TOUCH_N_SAMPLES 30
// capacitive val threshhold
#define TOUCH_THRESHOLD 70
// number of samples above threshold to trigget the touch button
#define TOUCH_COUNTS 20

CapacitiveSensor touch_btn = CapacitiveSensor(TOUCH_SEND_PIN, TOUCH_RECV_PIN);

void CSread();

void setup() {
  Serial.begin(115200);
  Serial.println("Setup");
}

void loop() { CSread(); }

unsigned long csSum;

void CSread() {
  long cs = touch_btn.capacitiveSensor(TOUCH_N_SAMPLES);

  if (cs > TOUCH_THRESHOLD) {
    csSum++;
    if (csSum >= TOUCH_COUNTS) {
      Serial.print("Touch");
      if (csSum > 0) {
        csSum = 0;
      }                              // Reset
      touch_btn.reset_CS_AutoCal();  // Stops readings
    }
  } else {
    csSum = 0;  // Timeout caused by bad readings
  }
}