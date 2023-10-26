#include <stdint.h>
#include "Arduino.h"

#ifndef LED_H
#define LED_H

#define FAST_FLASH_TIME 4000 //4ms
#define SLOW_FLASH_TIME 12000 // 12ms

class LED {
  private:
    
    uint8_t _pin;
    unsigned long _last_flash_time;
    unsigned long _interval;
    bool _state = false;
    

  public:
    
    LED(uint8_t pin);
    void LEDTrigger(bool trigger);
    void turnOff(void);
    void flashFast(void);
    void flashSlow(void);
};


#endif
