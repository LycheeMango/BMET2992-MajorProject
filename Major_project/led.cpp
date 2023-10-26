#include "led.h"

LED::LED(uint8_t pin) {
  uint8_t _pin = pin;
  unsigned long _last_flash_time = 0;
  unsigned long _interval = 0;
  bool _state = false;
}

void LED::LEDTrigger(bool trigger) {
  if (trigger==true) {
    digitalWrite(_pin, HIGH); 
    _state = true;
  }
  else {
    digitalWrite(_pin, LOW); 
    _state = false;
  }
}

void LED::turnOff(void) {
  digitalWrite(_pin, LOW); 
  _state = false;
}

void LED::flashFast(void) {
  _interval = FAST_FLASH_TIME;
  unsigned long current_time = micros();
  if (current_time - _last_flash_time >= _interval) {
    _last_flash_time = current_time;
    LEDTrigger(!_state); // toggle state
  }
}
void LED::flashSlow(void) {
  _interval = SLOW_FLASH_TIME;
  unsigned long current_time = micros();
  if (current_time - _last_flash_time >= _interval) {
    _last_flash_time = current_time;
    LEDTrigger(!_state); // toggle state
  }
}
