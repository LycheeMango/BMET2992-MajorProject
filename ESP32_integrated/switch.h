/*
** Class to debounce switches.
**
** Author:  Greg Watkins
** Date:    27 Aug 2021
**
*/

#include <stdint.h>

#ifndef  __SWITCH__
class Switch
{
  private:
    uint8_t _id;        // the number of the switch
    uint8_t _threshold; // Threshold for consecutive state mismatch before decidig teh state f teh switch has changed.
    bool    _state;     // Current switch state
    bool    _changed;   // indicates a change of state.
    uint8_t _count;     // count of consecutive state mismatches.
      
  public:
    Switch(uint8_t  id, uint8_t threshold);
    bool update(bool value);
    bool state(void);
    uint8_t id(void);
    bool changed(void);
};



class Button: public Switch
{
  private:
    bool _latch;
    uint8_t _pin;
    bool _button_flag;
    bool _press_once;
    bool _press_n_hold;
    unsigned long _press_start_time;
  public:
    Button(uint8_t id, uint8_t threshold, uint8_t pin = 33);
    void setPin(uint8_t pin);
    bool getFlag(void);
    void test(void); //test this class
    void inputParse(void);
    bool ifPressOnce(void);
    bool ifPressAndHold(void);
    bool getLatch(void);
};


#define __SWITCH__
#endif
