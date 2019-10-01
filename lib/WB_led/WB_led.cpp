/**
 * @file WB_led.cpp
 * @author Michele Brunelli (brunellim94@gmail.com)
 * @brief Library to manage led strips
 * @version 0.1
 * @date 2019-05-19
 *
 * @copyright Copyright (c) 2019
 *
 */

#include <WB_led.h>

// Call the Adafruit_Neopixel contructor
WB_led::WB_led(uint16_t n, uint8_t pin, neoPixelType type)
    : Adafruit_NeoPixel(n, pin, type) {
  _counter = 0;
  _led_status = IDLE;
}

void WB_led::standby_eff_(Adafruit_NeoPixel& strip, uint8_t pause,
                          uint32_t interval) {
  // check if there is another effect running
  if (_led_status != WB_STANDBY) {
    return;
  } else {
  }
}

/**
 * @brief Standby effect with counter - useful used with a timer
 *
 * @param counter the counter used to run the effect
 * @param strip
 * @param max_bright maximum brightness
 * @param pause_counts pause counts at max_bright
 * @param interval_counts intervall between next effect repetition
 */
void standby_eff(Adafruit_NeoPixel& strip, int& counter, uint8_t max_bright,
                 uint8_t pause_counts, uint32_t interval_counts) {
  if (counter <= max_bright) {
    // increase brightness
    strip.setPixelColor(strip.numPixels() - 1, strip.Color(30, 144, 255));
    strip.setBrightness(counter);
    strip.show();

  } else if (counter < max_bright + pause_counts) {
    // pause cycles at maximum brightness
  } else if (counter <= 2 * max_bright + pause_counts) {
    // decrease brightness
    strip.setPixelColor(strip.numPixels() - 1, strip.Color(30, 144, 255));
    strip.setBrightness(2 * max_bright + pause_counts - counter);
    strip.show();
  } else if (counter < (int)(2 * max_bright + pause_counts + interval_counts)) {
    // pause before next cycle
  } else {
    // reset counter and restart the cycle
    counter = 0;
    return;
  }
}

// Buffer used to store brightness values
extern uint8_t bright_buff[];

// TODO something don't work
void charging_eff(Adafruit_NeoPixel& strip, int& counter, uint8_t solar_perc,
                  uint32_t interval_counts) {
  double alpha = counter * PHASE_STEP - PI;

  // Get number of leds of strip
  uint16_t n_leds = strip.numPixels();

  uint16_t solar_leds = round(solar_perc * n_leds / 100);

  // interval between successive "tails"
  if (alpha < -PI) {
    // do nothing
  } else {
    // normal leds green
    for (uint16_t i = 0; i < n_leds - solar_leds; i++) {
      strip.setPixelColor(
          i, strip.Color(0, bright_buff[(i + counter) % n_leds], 0));
    }

    // solar leds yellow
    for (uint16_t i = n_leds - solar_leds; i < n_leds; i++) {
      strip.setPixelColor(i,
                          strip.Color(bright_buff[(i + counter) % n_leds],
                                      bright_buff[(i + counter) % n_leds], 0));
    }

    if (alpha > (PI + n_leds * PHASE_STEP)) {
      // reset counter
      counter = -interval_counts;
      return;

    } else if (alpha > PI) {
      // empty cycle
      bright_buff[counter % n_leds] = 0;

    } else {
      // color cycle
      bright_buff[counter % n_leds] = 255 * (cos(alpha) + 1) / 2;
    }

    strip.show();
  }
}

/**
 * @brief Update buffer with brightness for charging effect. Created to be used
 * with a timer, you need to pass a counter and increment it
 *
 * @param buff the buffer that contain the brightness values
 * @param buff_size
 * @param counter
 * @param interval_counts counts to wait before next effect cycle
 */
void charging_buff(uint8_t buff[], uint16_t buff_size, int& counter,
                   uint32_t interval_counts) {
  double alpha = counter * PHASE_STEP - PI;

  // interval between successive "tails"
  if (alpha < -PI) {
    // do nothing
    return;
  } else {
    uint8_t new_bright = 0;
    if (alpha > (PI + buff_size * PHASE_STEP)) {
      // reset counter
      counter = -interval_counts;
      return;

    } else if (alpha > PI) {
      // empty cycle
      new_bright = 0;

    } else {
      // color cycle
      new_bright = 255 * (cos(alpha) + 1) / 2;
    }

    // update buffer
    for (uint16_t i = buff_size - 1; i > 0; i--) {
      buff[i] = buff[i - 1];
    }
    // insert new value
    buff[0] = new_bright;
  }
}

/**
 * @brief Calculate brightness for standby effect. Created to be used
 * with a timer, you need to pass a counter and increment it
 *
 * @param counter
 * @param duration_counts
 * @param pause_counts
 * @param interval_counts
 * @return double
 */
double standby_buff(int& counter, uint32_t duration_counts,
                    uint8_t pause_counts, uint32_t interval_counts) {
  if (counter <= duration_counts / 2) {
    // increase brightness
    return (double)counter / duration_counts / 2;

  } else if (counter < duration_counts / 2 + pause_counts) {
    // pause cycles at maximum brightness
    return 1;
  } else if (counter <= duration_counts + pause_counts) {
    // decrease brightness
    return ((double)duration_counts + pause_counts - counter) /
           duration_counts / 2;
  } else if (counter <
             (int)(duration_counts + pause_counts + interval_counts)) {
    // pause before next cycle
    return 0;
  } else {
    // reset counter and restart the cycle
    counter = 0;
    return 0;
  }
}