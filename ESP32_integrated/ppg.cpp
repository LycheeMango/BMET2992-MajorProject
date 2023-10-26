#include "ppg.h"


PPG::PPG(int threshold, uint8_t low_pulse_threshold, uint8_t high_pulse_threshold) {


  // Adaptive threshold variables
  int _threshold = threshold; // set by user or default 1600

  // Heart rate calculation variables
  uint8_t _low_pulse_threshold = low_pulse_threshold; 
  uint8_t _high_pulse_threshold = high_pulse_threshold; 

  // OPTIONAL FEATURES:
  uint8_t _poor_recording_ceil = 200;// if detected heart rate is above 200 BPM
  uint8_t _poor_recording_floor = 10; // or if detected heart rate is extremely low (<10 BPM)
}

uint16_t PPG::getSensorReading() {
  return _sensor_reading;  
}
bool PPG::getPulseTrain(){
  return _current_PT_state; 
}

float PPG::getPulseRate() {
  if (_heart_rate < _low_pulse_threshold) {
    if (_heart_rate < _poor_recording_floor) {
      _pulse_warning = POOR_RECORDING; //  
    }
    else
    {
      _pulse_warning = LOW_PULSE_WARNING;
    }
  }
  else if (_heart_rate > _high_pulse_threshold) {
    if (_heart_rate > _poor_recording_ceil) {
      _pulse_warning = POOR_RECORDING; //  
    }
    else
    {
      _pulse_warning = HIGH_PULSE_WARNING;
    }
  }
  return _heart_rate;  
}

uint8_t PPG::getPulseWarning() {
  return _pulse_warning;  
}

void PPG::run() {
  
  _sensor_reading = analogRead(SENSOR_PIN);  

//********************************************************variable threshold *********************
  // calculate threshold 
  _adaptive_buffer[_buffer_idx++] = _sensor_reading; // increments the buffer index each time after this line in the loop runs
  if (_buffer_idx >= BUFFER_LENGTH) 
  {
    _buffer_idx = 0; // if the index reaches over 100 (set buffer length), the index goes back to 0 and values are overwritten again starting from 0 
  }
  
  // find max and min by comparing new values with the old ones, whichever one's bigger/smaller stays
  _max_val = 0;
  _min_val = 4095; // 12-bit ADC (10^12 -1)????????????????????
  
  for (int idx = 0; idx < BUFFER_LENGTH; idx++) // loops through every value in the buffer and compares with max/min
  {
    _max_val = max(_max_val, _adaptive_buffer[idx]); // compares the two values and takes the bigger one
    _min_val = min(_min_val, _adaptive_buffer[idx]);
    // apply and define the threshold: 80% betweenmax and min
    _threshold = ((_max_val-_min_val)*0.8 + _min_val); 
  }
  
  // print data  
  Serial.printf(" inputvalue:%d\n thresholdval: %d\n", _sensor_reading, _threshold);
//  Serial.printf("min:%d\n ,max: %d\n", _min_val, _max_val); // also print the detected local min and max

//********************************************************** pulse range & rate **************
  // Create a boolean signal based on the threshold
  _threshold_exceeded = (_sensor_reading > _threshold);
  
  // Update pulse train states
  _old_PT_state = _current_PT_state;
  _current_PT_state = _threshold_exceeded; //update both based on threshold , in case where it's decreasing:
                                           //threshold exceeds = false: old PT state becomes 1 but current PT state becomes 0
  
  // Detect when threshold is exceeded (0 to 1 transition)
  if (_current_PT_state==1 && _old_PT_state==0)
  {
    // Calculate the pulse period
    unsigned long currentTime = millis(); // takes the time when the threshold is exceeded
    _pulsePeriod = currentTime - _pulseStartTime; // calculates how long it is between each detected pulse 
    _pulseStartTime = currentTime; //update pulse start time so it becomes a time marker for the last pulse
  
    // Calculate heart rate in BPM
    if (_pulsePeriod > 0) 
    {
      _heart_rate = 60000.0 / (float)_pulsePeriod;
      //Serial.printf("Heart rate: %.1f BPM\n", heart_rate);  // prints the BPM (don't need to print this in arduino?)
  
    } else 
    {
      _heart_rate = 0.0; // To avoid division by zero
    }
  }
}
