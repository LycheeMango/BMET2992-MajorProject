/*
** Class to debounce switches.
**
** Author:  Greg Watkins
** Date:    27 Aug 2021
**
*/
#include "Arduino.h"
#include "switch.h"


Switch::Switch(uint8_t id, uint8_t threshold = 10)
/*
** Constructor
** Initialise id and threshold. The default threshold value is 10.
**
*/
{
  _id        = id;
  _threshold = threshold;
  _state     = false;
  _changed   = false;
  _count     = 0;
}

uint8_t Switch::id(void)
/*
** Returns the switch id
*/
{
  return _id;
}

bool Switch::changed(void)
{
/*
** Returns whtehr the switch has changed state with the last update.
*/
  return _changed;
}

bool Switch::state(void)
/*
** returns the current switch state.
*/
{
  return _state;
}

bool Switch::update(bool value)
/*
** Updates the count of switch state mismatches. This method
** shoudl be called each time teh applciation samples the switch input.
**
** value -  the state read from the physical switch.
**
** returns - true if the switch has changed state. 
**          i.e. the threshold of state mismatches has 
**          been exceeded. Otherwise fale
*/
{
  if (value == _state)
  { // have a match. Decrement counter but only to 0
    if (_count > 0)
    {
      _count--;
    }
  }
  else
  {
    // have a mismatch. increment count.
    _count++;
  }

  if (_count >= _threshold)
  {
    _changed = true;
    _state = !_state;
    _count = 0;
  }
  else
    _changed = false;

  return _changed;
}


// testing::

Button::Button(uint8_t id, uint8_t threshold, uint8_t pin) : Switch(id, threshold) {
    _latch = false;
    _pin = pin;
    _button_flag = false;
    _press_once = false;
    _press_n_hold = false;
}


void Button::setPin(uint8_t pin)
{
  _pin = pin;
}

// test is for testing only, not implemented in main code
void Button::test(void)
{

//  Serial.println("\nButton::test\n");
  
  bool switchState = digitalRead(_pin);
//  Serial.printf("\nswitchState = %x\n", switchState);
  if (update(switchState)) {
    
    // print in the serial port which switch has changed to what state
//    Serial.printf("Switch %x ->%x\n", id(), state());
    
    // >>> Insert code here so that each switch operates as a push on/push off switch rather than momentary contact
    // only change the latching signal if the switch is switched on
    if (state()) {
      // switch the latching state
      
      if (_button_flag == false) {
        _latch = !_latch;  
        Serial.printf("Latching signal state ->%x\n", _latch);
        _button_flag = true; // button is pressed once
      }
      else {
        _button_flag = false; // press and hold (button was previously pressed but a second rising edge is detected)  
      }

      
    }
  }
}


void Button::inputParse() {
  // Read the current state of the button
  bool buttonState = digitalRead(_pin);

  if (update(buttonState)) {
    // The button state has changed
    if (state()) {
      // Button is pressed
      if (!_button_flag) {
        // Button was not previously pressed (released)
        _button_flag = true; // Button is pressed once
        _press_start_time = millis(); // Record the time when the button was first pressed
      }
    } 
    else {
      // Button is released
        if (_button_flag) {
          // Button was previously pressed once
          if (millis() - _press_start_time >= 1000) {
            // Button was held for 1 second or more, change latching signal to 0
            _latch = false;
            _press_n_hold = true;
            _press_once = false;
          } else {
            // Button was pressed once (not held for 1 second), change latching signal to 1
            _latch = true;
            _press_once = true;
            _press_n_hold = false;
          }
          _button_flag = 0; // Reset the button press flag
        }
    }
  }

//  Serial.printf("\n _button_flag: %x \n", _button_flag);
  
//  Serial.printf("\n _press_once: %x \n", _press_once);
//  Serial.printf("\n _press_n_hold: %x \n", _press_n_hold);
}

bool Button::ifPressOnce() {
  return _press_once;  
}

bool Button::ifPressAndHold() {
  return _press_n_hold;  
}

bool Button::getLatch(void){
//  Serial.printf("\n _latch: %x \n", _latch);
  return _latch;
}
