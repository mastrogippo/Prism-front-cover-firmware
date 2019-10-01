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

#ifndef PANEL_CONFIG_HPP
#define PANEL_CONFIG_HPP

// WS2812 led led_strip
#define LED_STRIP_PIN 3

// capacitive button pin
#define CAP1203_ALERT_PIN PCINT10
#define CAP1203_SDA_PIN PC4
#define CAP1203_SCL_PIN PC5

// debounce interval in ms
#define TOUCH_DEBOUNCE_INTERVAL 200
#define TOUCH_SEND_PIN A2
#define TOUCH_RECV_PIN A3
// high number of sample lead to a better resolution, but with slower response
#define TOUCH_N_SAMPLES 30
// capacitive val threshhold
#define TOUCH_THRESHOLD 100
// number of samples above threshold to trigget the touch button
#define TOUCH_COUNTS 20

// number of leds to use for charging effect
#define N_LEDS_EFFECT 5
#define LEDS_OFFSET 2
#define N_LEDS N_LEDS_EFFECT + LEDS_OFFSET

// timer step (ms)
#define TIMER_STEP 22

// EFFECTS TIMINGS
#define STANDBY_INTERV 300
#define CHARGE_INTERV 0

#define STANDBY_DURATION_COUNTS 2000 / TIMER_STEP
#define STANDBY_PAUSE_COUNTS STANDBY_PAUSE / TIMER_STEP
#define STANDBY_INTERV_COUNTS STANDBY_INTERV / TIMER_STEP

#define TOP_BOTTOM 0
#define BOTTOM_TOP 1

#define CHARGING_EFF_DIR TOP_BOTTOM

#define LED_BRIGHTNESS 100

// number of brightness levels for brightness command
#define BRIGHTNESS_LEVELS 9

// Knock sensor
// toneAC lib use only pins 9 & 10 for buzzer
#define KNOCK_ANALOG_PIN A0
#define KNOCK_PIN1 9   // do not change!
#define KNOCK_PIN2 10  // do not change!
#define KNOCK_THRESH 50

// Tone
#define NOTE_B5 988
#define NOTE_E6 1319
#define NOTE_DURATION 100
#define NOTE_WAIT_TIME 50

#endif