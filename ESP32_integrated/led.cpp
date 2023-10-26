#include "led.h"

LED::LED(int pin) {
  int _pin = pin;
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
  Serial.print("off");
}

bool LED::getState(void){
  return _state;
}


void LED::setState(bool state) {
  _state = state;  
}
