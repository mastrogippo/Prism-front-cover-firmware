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
#include <TimerOne.h>
#include <WB_led.h>
#include <Wire.h>

// WS2812 led led_strip
#define LED_STRIP_PIN 6

#define N_LEDS 12

Adafruit_NeoPixel led_strip(N_LEDS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

// max value for slider, configured inside CPT
#define SLIDER_MAX_VAL 100

void led_ISR();

// timer step (ms)
#define TIMER_STEP 22

// EFFECTS TIMINGS

#define STANDBY_INTERV 300
#define CHARGE_INTERV 0

#define STANDBY_DURATION_COUNTS 2000 / TIMER_STEP
#define STANDBY_PAUSE_COUNTS STANDBY_PAUSE / TIMER_STEP
#define STANDBY_INTERV_COUNTS STANDBY_INTERV / TIMER_STEP

#define LED_BRIGHTNESS 200

// number of brightness levels for brightness command
#define BRIGHTNESS_LEVELS 5

// buffer used for charging effect
uint8_t strip_buff[N_LEDS / 2] = {0};

// start with "standby" status
char status[2] = {'s', 's'};
#define VALID_CMD_N 4
char valid_cmd_arr[VALID_CMD_N] = {'o', 'c', 's', 'b'};
uint8_t status_param = 0;

// buffer to store last cmd
char cmd_buff[3] = {0, 's', 0};
bool new_cmd = false;

int ISR_counter = 0;
int ISR_counter_slow = 0;

void setup() {
  Serial.begin(9600);
  // set timeout for readBytesUntil()
  Serial.setTimeout(500);

  pinMode(10, OUTPUT);  // DEBUG

  // Strips initialization
  led_strip.begin();
  led_strip.show();  // Turn OFF all pixels ASAP
  led_strip.setBrightness(LED_BRIGHTNESS);

  // timer step goes in us
  Timer1.initialize(TIMER_STEP * 1000);
  Timer1.attachInterrupt(led_ISR);

  // delay(3000);
  Serial.println("Setup Done");
}

void loop() {
  // Check for incoming command
  if (Serial.available() > 0) {
    char in_buff[8] = {};
    // Serial.readBytes(in_buff, 3);

    Serial.readBytesUntil('z', in_buff, 4);

    // Serial.println(in_buff);  // DEBUG

    int status_index = in_buff[0] - '0';

    // chech status index
    if (status_index != 0 && status_index != 1) {
      // invalid index
      return;
    }

    // Index is valid

    // check command
    bool cmd_is_valid = false;
    for (int i = 0; i < VALID_CMD_N; i++) {
      if (in_buff[1] == valid_cmd_arr[i]) {
        cmd_is_valid = true;
        break;
      }
    }

    // invalid command
    if (!cmd_is_valid) {
      return;
    }

    // Command and index are valid!

    // set param only if needed
    if (in_buff[1] == 'c') {
      // check if param is valid
      int param = in_buff[2] - '0';

      if (param < 0 || param > 9) {
        // invalid param
        return;
      }

      // char is converted to int
      cmd_buff[2] = param;
    }
    // Off command
    else if (in_buff[1] == 'o') {
      if (status_index == 0) {
        for (int16_t i = 0; i < (N_LEDS / 2); i++) {
          led_strip.setPixelColor(i, led_strip.Color(0, 0, 0));
        }
      }
      if (status_index == 1) {
        for (int16_t i = N_LEDS / 2; i < N_LEDS; i++) {
          led_strip.setPixelColor(i, led_strip.Color(0, 0, 0));
        }
      }
    }
    // Brightness command
    else if (in_buff[1] == 'b') {
      int param = in_buff[2] - '0';

      // check if param is valid
      if (param < 0 || param > BRIGHTNESS_LEVELS) {
        // invalid param
        return;
      }

      // char is converted to int
      cmd_buff[2] = param;
    }

    // Command valid! Save to buffer
    new_cmd = true;
    cmd_buff[0] = status_index;
    cmd_buff[1] = in_buff[1];
  }

  // update status only on counter reset
  if (new_cmd) {
    int status_index = cmd_buff[0];

    // Wait for the half we want to update to reset its counter
    if (status[status_index] == 's' && ISR_counter != 0) {
      return;
    }
    if (status[status_index] == 'c' && ISR_counter_slow != 0) {
      return;
    }

    // Here the half to update is clear
    // Set it off and wait for the other half to reset

    // old status needed for brightness command, to restore previous effect
    char old_status = status[status_index];

    noInterrupts();
    status[status_index] = 'o';
    interrupts();

    if (status[(status_index + 1) % 2] == 's') {
      while (ISR_counter != 0) {
        delay(50);
      }
    } else if (status[(status_index + 1) % 2] == 'c') {
      while (ISR_counter_slow != 0) {
        delay(50);
      }
    }

    // Here both half are off

    if (cmd_buff[1] == 'b') {
      int new_level = cmd_buff[2];
      led_strip.setBrightness((double)LED_BRIGHTNESS / BRIGHTNESS_LEVELS *
                              new_level);

      status[status_index] = old_status;

      new_cmd = false;
      // do not update status
      return;
    }

    noInterrupts();

    // update current status
    status[status_index] = cmd_buff[1];
    status_param = cmd_buff[2];

    interrupts();

    new_cmd = false;
  }
}

void led_ISR() {
  // unsigned long prev_time = millis();  // DEBUG

  ISR_counter++;

  // STANDBY EFFECT
  if (status[0] == 's' || status[1] == 's') {
    // calculate brightness scaling factor
    double bright_scaling =
        (double)standby_buff(ISR_counter, STANDBY_DURATION_COUNTS,
                             STANDBY_PAUSE_COUNTS, STANDBY_INTERV_COUNTS);

    // update the led brightness
    if (status[0] == 's') {
      led_strip.setPixelColor(
          N_LEDS / 2 - 1,
          led_strip.Color(30 * bright_scaling, 144 * bright_scaling,
                          255 * bright_scaling));
    }
    if (status[1] == 's') {
      led_strip.setPixelColor(
          N_LEDS / 2, led_strip.Color(30 * bright_scaling, 144 * bright_scaling,
                                      255 * bright_scaling));
    }
  }

  // CHARGING EFFECT
  // do the calculation only once for both half of the strip
  if (status[0] == 'c' || status[1] == 'c') {
    // Time interval of this routine is 4 times bigger the timer interval
    if (ISR_counter % 4 == 0) {
      ISR_counter_slow++;

      // Divide pause interval by 5 because this timer is 5 times slower than
      // the main timer
      charging_buff(strip_buff, N_LEDS / 2, ISR_counter_slow,
                    CHARGE_INTERV / (TIMER_STEP * 5));

      // uint16_t solar_leds = round(status_param * (N_LEDS / 2) / 100);

      // update left half
      // normal leds green
      if (status[0] == 'c') {
        for (int16_t i = 0; i < (N_LEDS / 2) - status_param; i++) {
          led_strip.setPixelColor(i, led_strip.Color(0, strip_buff[i], 0));
        }

        // solar leds yellow
        for (int16_t i = (N_LEDS / 2) - status_param; i < (N_LEDS / 2); i++) {
          led_strip.setPixelColor(
              i, led_strip.Color(strip_buff[i], strip_buff[i], 0));
        }
      }

      // update right half
      // normal leds green
      if (status[1] == 'c') {
        // right half must be updated in reverse order
        for (int16_t i = 0; i < (N_LEDS / 2) - status_param; i++) {
          led_strip.setPixelColor(N_LEDS - 1 - i,
                                  led_strip.Color(0, strip_buff[i], 0));
        }

        // solar leds yellow
        for (int16_t i = (N_LEDS / 2) - status_param; i < (N_LEDS / 2); i++) {
          led_strip.setPixelColor(
              N_LEDS - 1 - i, led_strip.Color(strip_buff[i], strip_buff[i], 0));
        }
      }

      // reset main counter only if both half is in charging status
      if (status[0] == 'c' && status[1] == 'c') {
        if (ISR_counter_slow == 0) {
          // -1 so the next cycle the ISR will increment the counter to 0
          ISR_counter = -1;
        }
      }
    }
  }

  led_strip.show();

  // Serial.println("ISR dur: " + String(millis() - prev_time));  // DEBUG
}