// Prism front cover
// Copyright (C) 2019 Mastro Gippo - Michele Brunelli
//
// This program is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// this program.  If not, see <http://www.gnu.org/licenses/>.

// For CPT I2C info see section 4.2 of datasheet

// Connect piezo to pin 9 & 10, toneAC lib use only these pins

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <CapacitiveSensor.h>
#include <MsTimer2.h>
#include <SparkFun_CAP1203.h>
#include <WB_led.h>
#include <Wire.h>
#include <toneAC.h>
#include "pins_arduino.h"  // Arduino pre-1.0 needs this

#include "config.hpp"
#include "prism_debug.hpp"

void led_ISR();
void next_mode();
void btn_sound();
void standby_led_eff();
void charging_led_eff();
void update_led_eff();
void btn(int n, int led_strip_index);
void cs_read();

uint8_t readCapacitivePin(int pinToMeasure);
bool knocked(uint8_t analog_pin, int threshold);

// CAP1203 touch;
CapacitiveSensor touch_btn = CapacitiveSensor(TOUCH_SEND_PIN, TOUCH_RECV_PIN);

Adafruit_NeoPixel led_strip(N_LEDS_EFFECT + LEDS_OFFSET, LED_STRIP_PIN,
                            NEO_GRB + NEO_KHZ800);

// buffer used for charging effect
uint8_t strip_buff[N_LEDS_EFFECT] = {0};
uint8_t mode = 0;

// start with "standby" status
char status = 's';
char valid_cmd_arr[] = {'o', 'c', 's', 'b', 'e', 'E', 'w', 'i', 'u'};
const uint32_t valid_cmd_n = sizeof(valid_cmd_arr) / sizeof(*valid_cmd_arr);
uint8_t status_param = 0;

// buffer to store last cmd
char cmd_buff[2] = {'s', 0};
bool new_cmd = false;

int ISR_counter = 0;
int ISR_counter_slow = 0;

unsigned long prev_sound = 0;

// 0 = bottom to top, 1 = top to bottom
uint8_t charging_eff_dir = 1;

// touch
uint16_t csCount = 0;
bool btn_touched = false;

bool isPaused = false;
bool isTouched = false;

typedef enum : uint16_t {
  PRISM_NORMAL,
  PRISM_NO_INTERNET,
  PRISM_OVERHEAT,
} PrismStatus;

PrismStatus prismStatus = PRISM_NORMAL;

void setup() {
  //Serial.begin(57600);
  Serial.begin(9600);
  pinMode(A6, INPUT);

  // set timeout for readBytesUntil()
  Serial.setTimeout(500);

  // Strips initialization
  led_strip.begin();
  led_strip.show();  // Turn OFF all pixels ASAP
  led_strip.setBrightness(LED_BRIGHTNESS);
  led_strip.clear();

  // Timer1 used by toneAC library - do not use it
  MsTimer2::set(TIMER_STEP, led_ISR);
  MsTimer2::start();

  Wire.begin();

  // Touch recalibration
  touch_btn.reset_CS_AutoCal();

  DEBUGLN("Setup Done");
}

void loop() {
  cs_read();

  // Sound ended, re-enable interrupt
  if (isPaused &&
      ((millis() - prev_sound) > (NOTE_DURATION + NOTE_WAIT_TIME))) {
    isPaused = false;
  }

  // Check touch and knock only if sound is ended
  if (!isPaused) {
    // Touch BTN
    // btn already touched

    if (isTouched) {
      // btn was already pressed, check if it is released
      if (!btn_touched) {
        Serial.println("R0");

        btn_sound();
        //next_mode();
        isTouched = false;
      }
    }

    // new btn touch
    else if (!isTouched && btn_touched) {
      DEBUGLN("Touch");
      isTouched = true;
      Serial.println("T0");
    }
    // Knock sensor
    else if (knocked(KNOCK_ANALOG_PIN, KNOCK_THRESH)) {
      DEBUGLN("Knock");
      // send k + number bw 0 and 9, based on knock sequence recognized
      // TODO recognize knock sequence
      Serial.println("K0");
      btn_sound();
    }
  }
}

void led_ISR() {
  switch (status) {
    case 's':  // STANDBY
      led_strip.clear();

      // Button's led
      led_strip.setPixelColor(0, led_strip.Color(0, 0, 200));
      standby_led_eff();
      break;

    case 'c':  // CHARGING
      // Button's led
      if (status_param > 0) {
        // Yellow if some energy from solar
        led_strip.setPixelColor(0, led_strip.Color(255, 255, 0));
      } else {
        // Green if all energy from the grid
        led_strip.setPixelColor(0, led_strip.Color(0, 255, 0));
      }
      charging_led_eff();
      break;

    case 'o':  // OFF
      for (int16_t i = 0; i < (N_LEDS_EFFECT + LEDS_OFFSET); i++) {
        led_strip.setPixelColor(i, led_strip.Color(0, 0, 0));
      }
      break;

    case 'e': {  // ERROR

      led_strip.clear();
      // Button's led
      led_strip.setPixelColor(0, led_strip.Color(255, 0, 0));
      // param from 1 to 5 -> light X led red
      // int param_int = status_param - '0';
      if (status_param > 0 && status_param <= N_LEDS_EFFECT) {
        for (int i = 0; i < status_param; i++) {
          led_strip.setPixelColor(LEDS_OFFSET + i, led_strip.Color(255, 0, 0));
        }
      }
    } break;

    case 'E': {
      // Button's led
      led_strip.setPixelColor(0, led_strip.Color(255, 0, 0));

      int temp_param = status_param;
      for (int i = 0; i < N_LEDS_EFFECT; i++) {
        if ((temp_param % 2) == 1) {
          led_strip.setPixelColor(LEDS_OFFSET + i, led_strip.Color(255, 0, 0));
        } else {
          led_strip.setPixelColor(LEDS_OFFSET + i, led_strip.Color(0, 0, 0));
        }

        temp_param = temp_param / 2;
      }
    } break;

    case 'u':
      // all leds purple
      led_strip.setPixelColor(0, led_strip.Color(195, 0, 255));
      update_led_eff();
      break;

    default:
      DEBUGLN("Invalid status");
      break;
  }

  switch (prismStatus) {
    case PRISM_OVERHEAT:
      // set upper led red
      led_strip.setPixelColor(N_LEDS - 1, led_strip.Color(255, 0, 0));
      break;

    case PRISM_NO_INTERNET:
      // set upper led purple
      led_strip.setPixelColor(N_LEDS - 1, led_strip.Color(195, 0, 255));
      break;

    case PRISM_NORMAL:
      // do nothing
      break;

    default:
      DEBUGLN("Invalid prism status");
      break;
  }

  led_strip.show();

  ISR_counter++;
}

// Read capacitive btn
void cs_read() {
  noInterrupts();
  long cs = touch_btn.capacitiveSensor(TOUCH_N_SAMPLES);
  interrupts();

  if (cs > TOUCH_THRESHOLD) {
    if (csCount >= TOUCH_COUNTS) {
      // Touch event!
      btn_touched = true;
    } else {
      csCount++;
    }
  } else {
    csCount = 0;  // Timeout caused by bad readings
    btn_touched = false;
  }
}

void next_mode() {
  noInterrupts();
  if (status == 's') {
    // charging - all energy from grid
    status = 'c';
    status_param = 0;

  } else if (status == 'c') {
    if (status_param == 0) {
      // charging - some energy from solar
      status_param = 3;
    } else {
      // standby
      status = 'e';
      status_param = 1;
    }
  } else if (status == 'e') {
    status = 's';
  }
  interrupts();
}

bool knocked(uint8_t analog_pin, int threshold) {
  pinMode(KNOCK_PIN2, INPUT);
  pinMode(KNOCK_PIN1, OUTPUT);
  digitalWrite(KNOCK_PIN1, LOW);

  int sensorReading = analogRead(analog_pin);

  // if the sensor reading is greater than the threshold:
  if (sensorReading >= threshold) {
    delay(10);
    pinMode(KNOCK_PIN2, OUTPUT);
    pinMode(KNOCK_PIN1, OUTPUT);

    return true;
  }

  return false;
}

void btn_sound() {
  isPaused = true;
  // Save starting time to wait after sound end without using delay()
  prev_sound = millis();

  toneAC(NOTE_E6, 5, NOTE_DURATION, true);  // it was 500
}

// Receive serial commands
// COMMAND FORMAT: 'a', status byte, param byte, strip index byte, 'z'
void serialEvent() {
  if (Serial.available() > 0) {
    char in_buff[6] = {};

    Serial.readBytes(in_buff, 5);

    DEBUG("Buff: ");
    DEBUGLN(in_buff);

    // check start and stop byte
    if (in_buff[0] != 'a' || in_buff[4] != 'z') {
      DEBUGLN("Invalid start or stop char");
      return;
    }

    // check command
    bool cmd_is_valid = false;
    for (unsigned int i = 0; i < valid_cmd_n; i++) {
      if (in_buff[1] == valid_cmd_arr[i]) {
        cmd_is_valid = true;
        break;
      }
    }
    if (!cmd_is_valid) {
      DEBUGLN("Invalid cmd!");
      return;
    }

    DEBUGLN("Valid status");

    char new_status = in_buff[1];
    char new_param = in_buff[2];

    // BRIGHTNESS
    if (new_status == 'b') {
      // convert to int
      int param_int = new_param - '0';

      // check if param is valid
      if (param_int < 0 || param_int > BRIGHTNESS_LEVELS) {
        // invalid param
        DEBUGLN("Invalid param");
        return;
      } else if (param_int == 0) {
        param_int = 1;
      }

      led_strip.setBrightness((double)LED_BRIGHTNESS / BRIGHTNESS_LEVELS *
                              param_int);

      // do not update status
      return;
    }
    // CHARGING
    else if (new_status == 'c') {
      int param_int = new_param - '0';

      // numbers bw 0 and 6 are valid
      if (param_int < 0 && param_int > N_LEDS_EFFECT) {
        // invalid param
        DEBUGLN("Invalid param");
        return;
      }

      // disable interrupts, status_param used by ISR
      noInterrupts();
      // char is converted to int
      status_param = new_param - '0';

      // Restart charging effect only if previous status is different
      if (status != 'c') {
        DEBUGLN("Reset charging count");
        ISR_counter = 0;
        ISR_counter_slow = 0;

        // TODO try with led_strip.clear()
        // clear strip_buff
        for (int i = 0; i < N_LEDS_EFFECT; i++) {
          strip_buff[i] = 0;
        }
      }
    }
    // ERROR
    else if (new_status == 'e') {
      cmd_is_valid = false;
      // convert to int
      int param_int = new_param - '0';
      if (param_int > 0 && param_int <= N_LEDS_EFFECT) {
        cmd_is_valid = true;
      }

      if (!cmd_is_valid) {
        return;
      } else {
        status_param = param_int;
      }
    }
    // ERROR-2
    else if (new_status == 'E') {
      // convert to int
      int param_int = new_param;
      // b11111 = 31 dec
      // use number from 0 to 9 and then char from 'a' to 'w' (10 to 31)
      if ((param_int >= (int)'0' && param_int <= (int)'9')) {
        status_param = new_param - '0';
      } else if ((param_int >= (int)'a' && param_int <= (int)'v')) {
        status_param = new_param - 'a' + 10;
      } else {
        DEBUGLN("Invalid param");
        return;
      }
    }
    // OVERHEAT
    else if (new_status == 'w') {
      int param_int = new_param - '0';

      if (param_int == 1) {
        DEBUGLN("OVERHEAT!");
        prismStatus = PRISM_OVERHEAT;
      } else if (param_int == 0) {
        prismStatus = PRISM_NORMAL;
      } else {
        DEBUGLN("Invalid status param");
      }

      // no need to save status and param
      return;
    }
    // INTERNET STATUS
    else if (new_status == 'i') {
      int param_int = new_param - '0';

      if (param_int == 0) {
        DEBUGLN("No Internet!");
        prismStatus = PRISM_NO_INTERNET;
      } else if (param_int == 1) {
        prismStatus = PRISM_NORMAL;
      } else {
        DEBUGLN("Invalid status param");
      }

      // no need to save status and param
      return;
    }
    // UPDATE
    else if (new_status == 'u') {
      // Restart effect only if previous status is different
      if (status != 'u') {
        DEBUGLN("Reset effect count");
        ISR_counter = 0;

        status = new_status;
      }
      return;
    }

    // disable interrupts, variables used by ISR
    noInterrupts();
    // set_new_status
    status = new_status;
    // re-enable interrupts
    interrupts();
    DEBUGLN("Valid cmd!");
  }
}

// Led charging effect
void charging_led_eff() {
  // Time interval of this routine is 5 times bigger the timer interval
  if (ISR_counter % 4 == 0) {
    ISR_counter_slow++;

    // Divide pause interval by 5 because this timer is 5 times slower than
    // the main timer
    charging_buff(strip_buff, N_LEDS_EFFECT, ISR_counter_slow,
                  CHARGE_INTERV / (TIMER_STEP * 5));
    if (charging_eff_dir == 0) {
      // Scroll from bottom to top
      // normal leds green
      for (int16_t i = LEDS_OFFSET; i < (N_LEDS - status_param); i++) {
        led_strip.setPixelColor(
            i, led_strip.Color(0, strip_buff[i - LEDS_OFFSET], 0));
      }

      // solar leds yellow
      for (int16_t i = (N_LEDS - status_param); i < N_LEDS; i++) {
        led_strip.setPixelColor(
            i, led_strip.Color(strip_buff[i - LEDS_OFFSET],
                               strip_buff[i - LEDS_OFFSET], 0));
      }
    } else if (charging_eff_dir == 1) {
      // Scroll from top to bottom
      // solar leds yellow
      for (int16_t i = N_LEDS - 1; i >= (N_LEDS - status_param); i--) {
        led_strip.setPixelColor(i,
                                led_strip.Color(strip_buff[N_LEDS - 1 - i],
                                                strip_buff[N_LEDS - 1 - i], 0));
      }
      // normal leds green
      // ERROR the hidden led light up
      for (int16_t i = (N_LEDS - status_param) - 1; i >= LEDS_OFFSET; i--) {
        led_strip.setPixelColor(
            i, led_strip.Color(0, strip_buff[N_LEDS - 1 - i], 0));
      }
    }
    // If slow counter is reset, reset also the main counter to avoid
    // overflow
    if (ISR_counter_slow < 0) {
      ISR_counter = 0;
    }
  }
}

// Led standby effect
void standby_led_eff() {
  // calculate brightness scaling factor
  double bright_scaling =
      (double)standby_buff(ISR_counter, STANDBY_DURATION_COUNTS,
                           STANDBY_PAUSE_COUNTS, STANDBY_INTERV_COUNTS);

  // update the led brightness (the standby led is the first one)
  led_strip.setPixelColor(
      LEDS_OFFSET, led_strip.Color(30 * bright_scaling, 144 * bright_scaling,
                                   255 * bright_scaling));
}

void update_led_eff() {
  // calculate brightness scaling factor
  double bright_scaling =
      (double)standby_buff(ISR_counter, STANDBY_DURATION_COUNTS,
                           STANDBY_PAUSE_COUNTS, STANDBY_INTERV_COUNTS);

  // update the led brightness
  // calculation to add a small offset - the leds doesn't turn off completely
  for (int i = LEDS_OFFSET; i < N_LEDS; i++) {
    led_strip.setPixelColor(
        i, led_strip.Color(195 * (bright_scaling + 0.11) / 1.1, 0,
                           255 * (bright_scaling + 0.11) / 1.1));
  }
}
