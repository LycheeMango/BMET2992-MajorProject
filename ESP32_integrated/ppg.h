#include <stdint.h>
#include "Arduino.h"

#ifndef PPG_H
#define PPG_H

#define BUFFER_LENGTH 50 // buffer length default = 50, two seconds
#define NORMAL 0 // for pulse warning signals
#define LOW_PULSE_WARNING 1
#define HIGH_PULSE_WARNING 2
#define POOR_RECORDING 3

#define SENSOR_PIN 25


class PPG {
  
  private:
    uint16_t _sensor_reading = 0; // sensor reading

    // Adaptive threshold variables
    int _threshold = 1600; // adaptive threshold, first tamporarily define the threshold, its value will change later accordingly, default = 1600
    uint16_t _adaptive_buffer[BUFFER_LENGTH];
    uint8_t _buffer_idx = 0; 
    uint16_t _max_val = 0;
    uint16_t _min_val = 4095;
    bool _threshold_exceeded = false;
    
    // Pulse Train State variables
    bool _old_PT_state = false;     // The previous pulse train state
    bool _current_PT_state = false; // The current pulse train state
    
    // Pulse period measurement variables
    unsigned long _pulseStartTime = 0; // Time when the pulse started
    unsigned long _pulsePeriod = 0;    // Pulse period in milliseconds
    
    // Heart rate calculation variables
    float _heart_rate = 0.0; // Heart rate in BPM
    uint8_t _pulse_warning = NORMAL; 
    uint8_t _low_pulse_threshold = 50;
    uint8_t _high_pulse_threshold = 100; 
    
    // low heart rate < 50 BPM 
    // high heart rate (No AFib) (100 - 150 BPM), 
    // Artial Fibrillation High Heart Rate (100-150 bpm) high heart rate 
    // High Heart Rate ( > 150 bpm) (taken from Apple Inc. ECG 2.0 App 510(k) specs.)

    // OPTIONAL FEATURES:
    uint8_t _poor_recording_ceil = 200; // if detected heart rate is above 200 BPM
    uint8_t _poor_recording_floor = 10; // or if detected heart rate is extremely low (<10 BPM)
      
  public:
    
    PPG (int threshold = 1600, uint8_t low_pulse_threshold = 50, uint8_t high_pulse_threshold = 100); // constructor
    uint16_t getSensorReading(); // get sensor analog reading
    bool getPulseTrain(); // get pulse train, rising edge for each detected pulse
    float getPulseRate(); // get pulse rate, which will detect for high, low pulse rate warning.
    uint8_t getPulseWarning(); // get pulse warning, 0 for normal, 1 for low pulse warning, 2 for high pulse warning, 3 = poor recording
    uint8_t resetPulseWarning(); // resets pulse warning to 0
    void run(); // run ppg measuring

};



#endif
