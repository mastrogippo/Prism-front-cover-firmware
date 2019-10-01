/**
 * @file main.cpp
 * @author Michele Brunelli (brunellim94@gmail.com)
 * @brief CPT touch sensor example
 * @version 0.1
 * @date 2019-05-16
 *
 * @copyright Copyright (c) 2019
 *
 */

// For CPT I2C info see section 4.2 of datasheet

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Wire.h>

// WS2812 led strip
#define SX_LED_PIN 6
#define RX_LED_PIN 7

#define N_LEDS 6

Adafruit_NeoPixel strip_sx(N_LEDS, SX_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_rx(N_LEDS, RX_LED_PIN, NEO_GRB + NEO_KHZ800);

// CPT112s
#define SDA_PIN 2
#define SCL_PIN 3
#define INT_PIN 4

byte buff[4];

#define EVENT_TYPE_MASK 0x0F
// max value for slider, configured inside CPT
#define SLIDER_MAX_VAL 100

void cptEvent();
void show_leds(Adafruit_NeoPixel& strip, int n_led, uint32_t color);

void setup() {
  Serial.begin(9600);

  /*
  Wire.begin();

  // CPT interrupt is active low
  attachInterrupt(digitalPinToInterrupt(INT_PIN), cptEvent, FALLING);
  */

  // Strips initialization
  strip_sx.begin();
  strip_sx.show();  // Turn OFF all pixels ASAP
  strip_sx.setBrightness(20);

  strip_rx.begin();
  strip_rx.show();  // Turn OFF all pixels ASAP
  strip_rx.setBrightness(20);

  Serial.println("Setup Done");
}

byte byte1 = 0x22;

void loop() {
  // put your main code here, to run repeatedly:

  for (int i = 0; i <= SLIDER_MAX_VAL; i++) {
    int leds_to_show = map(i, 0, SLIDER_MAX_VAL + 1, 0, 4);

    Serial.println(String(i) + ", " + String(leds_to_show));
    delay(200);
  }
}

void cptEvent() {
  // Read all 4 bytes sent by CPT
  Wire.readBytes(buff, 4);

  if ((buff[1] & EVENT_TYPE_MASK) == 2) {
    // Slider event
    uint16_t slider_pos = buff[2] << 8 + buff[3];

    // This way the slider_val will be divided exactly in N_LEDS equal interval
    int leds_to_show = map(slider_pos, 0, SLIDER_MAX_VAL + 1, 0, N_LEDS + 1);

    show_leds(strip_sx, leds_to_show, strip_sx.Color(255, 255, 255));
  }
}

void show_leds(Adafruit_NeoPixel& strip, int n_led, uint32_t color) {
  if (n_led == 0) {
    strip.clear();
  } else {
    strip.fill(color, 0, n_led);
  }

  strip.show();
}