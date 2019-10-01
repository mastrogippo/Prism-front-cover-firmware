/**
 * @file WB_led.h
 * @author Michele Brunelli (brunellim94@gmail.com)
 * @brief Library to manage led strips
 * @version 0.1
 * @date 2019-05-19
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef WB_LED_H
#define WB_LED_H

#include <Adafruit_NeoPixel.h>  //https://github.com/adafruit/Adafruit_NeoPixel

// Charging effect variables
#define PHASE_STEP 0.5

// Standby effect variables
// pause at maximum brightness (ms)
#define STANDBY_PAUSE 30

// Led status enum
enum led_status {
  IDLE = -1,
  WB_OFF,
  WB_STANDBY,
  WB_CHARGING,
  WB_BAR,
};

class WB_led : public Adafruit_NeoPixel {
 public:
  WB_led(uint16_t n, uint8_t pin, neoPixelType type);

  // TODO add speed param?
  void standby_eff_(Adafruit_NeoPixel& strip, uint8_t pause, uint32_t interval);
  void standby_stop();

 private:
  int _counter;
  led_status _led_status;
};

void standby_eff(Adafruit_NeoPixel& strip, int& counter, uint8_t max_bright,
                 uint8_t pause_counts, uint32_t interval_counts);

void charging_eff(Adafruit_NeoPixel& strip, int& counter, uint8_t solar_perc,
                  uint32_t interval_counts);

void charging_buff(uint8_t* buff, uint16_t buff_size, int& counter,
                   uint32_t interval_counts);

double standby_buff(int& counter, uint32_t duration_counts,
                    uint8_t pause_counts, uint32_t interval_counts);

#endif  // WB_LED_H
